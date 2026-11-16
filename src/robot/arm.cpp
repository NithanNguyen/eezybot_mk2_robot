#include "arm.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "ArmMotion.h"

#if ARM_OUTPUT_ENABLED
#include "ServoPwm.h"
#include "board_pins.h"
#endif

const char* binName(Bin b) {
  switch (b) {
    case Bin::GREEN:  return "GREEN";
    case Bin::RED:    return "RED";
    case Bin::WHITE:  return "WHITE";
    case Bin::REJECT: return "REJECT";
  }
  return "?";
}

namespace {

const uint32_t PERIOD_US = 20000;           // = armcfg::CONTROL_PERIOD_S
const uint32_t LATE_US = 2000;              // nhip coi la tre neu thuc day lech > 2 ms hoac tinh > 20 ms
const UBaseType_t TASK_PRIO = 3;            // cao hon loop() (Arduino: 1) de nhip 20 ms khong bi chan
const BaseType_t TASK_CORE = 1;             // cung loi voi loop(): khong chay song song, khong can khoa g_seq
const uint32_t TASK_STACK = 4096;           // byte

enum ReqKind : uint8_t { REQ_HOME = 0, REQ_SORT = 1 };
struct Req {
  uint8_t kind;
  uint8_t dest;  // motion::Dest
};

enum MsgKind : uint8_t { MSG_CMD = 0, MSG_DONE = 1 };
struct Msg {
  uint8_t kind, phase, req, dest;
  uint8_t status;
  uint32_t tick;      // MSG_CMD: so thu tu nhip trong chu trinh; MSG_DONE: so nhip
  float s[3], grip;   // MSG_CMD
  uint32_t cmaxUs;    // MSG_DONE
};

struct Timing {
  uint32_t ticks, active, cmaxUs, late, dropped;
  uint64_t csumUs;
  uint32_t jmaxUs;
  uint32_t sat;   // so lan mot xung bi kep vao gioi han (MG996R_MIN/MAX_US, SG90_MIN/MAX_US)
};

// Kenh xung: 0 de, 1 vai, 2 khuyu (MG996R), 3 kep (SG90).
const uint8_t NCH = 4;
const float PULSE_LO[NCH] = {(float)armcfg::MG996R_MIN_US, (float)armcfg::MG996R_MIN_US,
                             (float)armcfg::MG996R_MIN_US, (float)armcfg::SG90_MIN_US};
const float PULSE_HI[NCH] = {(float)armcfg::MG996R_MAX_US, (float)armcfg::MG996R_MAX_US,
                             (float)armcfg::MG996R_MAX_US, (float)armcfg::SG90_MAX_US};
// Kep chi phat xung khi da hieu chuan that (params.py GRIP_PULSE_MEASURED): so gia dinh co the ep hai ngon vao nhau.
const bool GRIP_OUTPUT = armcfg::GRIP_PULSE_MEASURED;

QueueHandle_t g_req = nullptr, g_msg = nullptr;
TaskHandle_t g_task = nullptr;
volatile bool g_running = false, g_trace = false;
portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
Timing g_tm = {};               // bao ve boi g_mux
motion::Command g_last = {};    // lenh nhip gan nhat, bao ve boi g_mux
float g_lastUs[NCH] = {};       // xung (da kep) tuong ung g_last, bao ve boi g_mux
motion::Sequencer g_seq;        // chi task dung sau begin()
volatile bool g_outReq = true;  // false sau "ARM OFF" (chot toi khi reset): task ngung xung
#if ARM_OUTPUT_ENABLED
ServoPwm g_servo[NCH];          // chi task dung sau begin()
bool g_outOn = false;           // chi task dung
#endif

motion::Dest toDest(Bin b) {
  switch (b) {
    case Bin::GREEN: return motion::Dest::GREEN;
    case Bin::RED:   return motion::Dest::RED;
    case Bin::WHITE: return motion::Dest::WHITE;
    default:         return motion::Dest::REJECT;
  }
}

const char* reqName(const Req& r) {
  if (r.kind == REQ_HOME) return "HOME";
  switch ((motion::Dest)r.dest) {
    case motion::Dest::RED:   return "RED";
    case motion::Dest::GREEN: return "GREEN";
    case motion::Dest::WHITE: return "WHITE";
    default:                  return "REJECT";
  }
}

// Goc khop -> do rong xung (lop do that, Mk2Kinematics::toPulseUs / gripPulseUs), kep vao gioi han tung kenh.
// Tra ve so kenh bi kep. Chay kho van tinh de ARM STATUS cho thay xung se phat.
uint32_t pulses(const motion::Command& c, float us[NCH]) {
  mk2kin::toPulseUs(c.q, us);
  us[3] = mk2kin::gripPulseUs(c.gripDeg);
  uint32_t n = 0;
  for (uint8_t i = 0; i < NCH; i++) {
    if (us[i] < PULSE_LO[i]) { us[i] = PULSE_LO[i]; n++; }
    else if (us[i] > PULSE_HI[i]) { us[i] = PULSE_HI[i]; n++; }
  }
  return n;
}

void output(const float us[NCH]) {
#if ARM_OUTPUT_ENABLED
  if (!g_outReq) {
    if (g_outOn) {
      for (uint8_t i = 0; i < NCH; i++) g_servo[i].off();
      g_outOn = false;
    }
    return;
  }
  for (uint8_t i = 0; i < 3; i++) g_servo[i].writeUs((uint32_t)lroundf(us[i]));
  if (GRIP_OUTPUT) g_servo[3].writeUs((uint32_t)lroundf(us[3]));
  g_outOn = true;
#else
  (void)us;  // chay kho: khong phat xung
#endif
}

void motionTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(PERIOD_US / 1000);
  TickType_t last = xTaskGetTickCount();
  int64_t prev = esp_timer_get_time();
  Req cur = {REQ_HOME, 0};
  uint32_t runTicks = 0, runCmax = 0;
  bool first = true;
  for (;;) {
    vTaskDelayUntil(&last, period);
    const int64_t t0 = esp_timer_get_time();
    // Nhip dau: khoang cach tu luc tao task khong phai mot chu ky -> khong tinh do lech.
    const int32_t dj = first ? 0 : (int32_t)(t0 - prev) - (int32_t)PERIOD_US;
    first = false;
    prev = t0;

    Req r;
    if (!g_seq.active() && xQueuePeek(g_req, &r, 0) == pdTRUE) {
      g_running = true;  // dat TRUOC khi lay khoi hang doi -> busy() khong bao gio sai
      xQueueReceive(g_req, &r, 0);
      cur = r;
      runTicks = runCmax = 0;
      if (r.kind == REQ_HOME) g_seq.startHome();
      else g_seq.startSort((motion::Dest)r.dest);
    }
    motion::Command c;
    const bool act = g_seq.tick(c);
    float us[NCH] = {};
    uint32_t sat = 0;
    if (act) {
      sat = pulses(c, us);
      output(us);
    } else if (!g_outReq) {
      output(us);  // "ARM OFF" khi dang dung yen: tat xung ngay
    }
    const uint32_t cus = (uint32_t)(esp_timer_get_time() - t0);
    const uint32_t jus = (uint32_t)(dj < 0 ? -dj : dj);

    portENTER_CRITICAL(&g_mux);
    g_tm.ticks++;
    if (jus > g_tm.jmaxUs) g_tm.jmaxUs = jus;
    if (jus > LATE_US || cus > PERIOD_US) g_tm.late++;
    if (act) {
      g_tm.active++;
      g_tm.csumUs += cus;
      if (cus > g_tm.cmaxUs) g_tm.cmaxUs = cus;
      g_tm.sat += sat;
      for (uint8_t i = 0; i < NCH; i++) g_lastUs[i] = us[i];
    }
    g_last = c;
    portEXIT_CRITICAL(&g_mux);

    if (act) {
      runTicks++;
      if (cus > runCmax) runCmax = cus;
      if (g_trace) {
        Msg m = {};
        m.kind = MSG_CMD;
        m.phase = c.phase;
        m.tick = runTicks;
        m.s[0] = c.servoDeg[0];
        m.s[1] = c.servoDeg[1];
        m.s[2] = c.servoDeg[2];
        m.grip = c.gripDeg;
        if (xQueueSend(g_msg, &m, 0) != pdTRUE) {
          portENTER_CRITICAL(&g_mux);
          g_tm.dropped++;
          portEXIT_CRITICAL(&g_mux);
        }
      }
    }
    if (g_running && !g_seq.active()) {
      Msg m = {};
      m.kind = MSG_DONE;
      m.req = cur.kind;
      m.dest = cur.dest;
      m.status = (uint8_t)g_seq.error();
      m.tick = runTicks;
      m.cmaxUs = runCmax;
      xQueueSend(g_msg, &m, pdMS_TO_TICKS(5));
      g_running = false;
    }
  }
}

bool sendReq(uint8_t kind, uint8_t dest) {
  if (!g_req) return false;
  const Req r = {kind, dest};
  return xQueueSend(g_req, &r, 0) == pdTRUE;
}

void upcase(char* s) {
  for (; s && *s; s++) *s = (char)toupper((unsigned char)*s);
}

}  // namespace

void Arm::begin() {
  g_req = xQueueCreate(4, sizeof(Req));
  g_msg = xQueueCreate(64, sizeof(Msg));
  // Khong co cam bien vi tri: coi tay dang o Home, kep mo. Tay that phai duoc dat ve Home truoc khi cap nguon servo.
  g_seq.reset(motion::homeJoint(), motion::gripOpenDeg());
#if ARM_OUTPUT_ENABLED
  // Kenh LEDC 0..3 (core 2.x), cung thu tu voi firmware calib. Chua phat xung cho toi nhip dau cua task.
  const uint8_t pin[NCH] = {pins::SERVO_J1_BASE, pins::SERVO_J2_SHOULDER, pins::SERVO_J3_ELBOW, pins::SERVO_GRIPPER};
  for (uint8_t i = 0; i < NCH; i++) {
    if (i == 3 && !GRIP_OUTPUT) continue;
    g_servo[i].attach(pin[i], i, (uint16_t)PULSE_LO[i], (uint16_t)PULSE_HI[i]);
    g_servo[i].off();
  }
#endif
  xTaskCreatePinnedToCore(motionTask, "motion", TASK_STACK, nullptr, TASK_PRIO, &g_task, TASK_CORE);
  Serial.printf("# arm: task 20 ms, ARM_OUTPUT_ENABLED=%d (%s)\n", ARM_OUTPUT_ENABLED,
                ARM_OUTPUT_ENABLED ? "phat xung" : "chay kho, khong phat xung servo");
  if (ARM_OUTPUT_ENABLED && !GRIP_OUTPUT)
    Serial.println("# arm: kep CHUA hieu chuan (GRIP_PULSE_MEASURED = false) -> khong phat xung GPIO33");
  goHome();
}

bool Arm::busy() const {
  return g_running || (g_req && uxQueueMessagesWaiting(g_req) > 0);
}

void Arm::goHome() {
  if (!sendReq(REQ_HOME, 0)) Serial.println("ERR ARM_QUEUE_FULL");
}

void Arm::sortTo(Bin bin) {
  if (!sendReq(REQ_SORT, (uint8_t)toDest(bin))) Serial.println("ERR ARM_QUEUE_FULL");
}

void Arm::update() {
  if (!g_msg) return;
  Msg m;
  while (xQueueReceive(g_msg, &m, 0) == pdTRUE) {
    if (m.kind == MSG_CMD) {
      Serial.printf("CMD,%lu,%s,%.4f,%.4f,%.4f,%.4f\n", (unsigned long)m.tick, motion::phaseName((motion::Phase)m.phase),
                    m.s[0], m.s[1], m.s[2], m.grip);
    } else {
      const Req r = {m.req, m.dest};
      uint32_t dropped;
      portENTER_CRITICAL(&g_mux);
      dropped = g_tm.dropped;
      portEXIT_CRITICAL(&g_mux);
      Serial.printf("ARM DONE %s status=%s ticks=%lu cmax_us=%lu dropped=%lu\n", reqName(r),
                    mk2kin::statusName((mk2kin::Status)m.status), (unsigned long)m.tick, (unsigned long)m.cmaxUs,
                    (unsigned long)dropped);
    }
  }
}

bool Arm::handleCommand(char* tok[], int n) {
  if (n < 1 || strcmp(tok[0], "ARM") != 0) return false;
  for (int i = 1; i < n; i++) upcase(tok[i]);
  const char* sub = n >= 2 ? tok[1] : "";
  if (!strcmp(sub, "SORT") && n >= 3) {
    Bin b;
    if (!strcmp(tok[2], "GREEN")) b = Bin::GREEN;
    else if (!strcmp(tok[2], "RED")) b = Bin::RED;
    else if (!strcmp(tok[2], "WHITE")) b = Bin::WHITE;
    else if (!strcmp(tok[2], "REJECT")) b = Bin::REJECT;
    else {
      Serial.println("ERR ARM_BAD_BIN");
      return true;
    }
    if (busy()) {
      Serial.println("ERR ARM_BUSY");
      return true;
    }
    sortTo(b);
    Serial.printf("OK ARM SORT %s\n", binName(b));
  } else if (!strcmp(sub, "HOME")) {
    if (busy()) {
      Serial.println("ERR ARM_BUSY");
      return true;
    }
    goHome();
    Serial.println("OK ARM HOME");
  } else if (!strcmp(sub, "TRACE") && n >= 3) {
    g_trace = atoi(tok[2]) != 0;
    Serial.printf("OK ARM TRACE %d\n", g_trace ? 1 : 0);
  } else if (!strcmp(sub, "TIMING")) {
    Timing t;
    portENTER_CRITICAL(&g_mux);
    if (n >= 3 && !strcmp(tok[2], "RESET")) g_tm = Timing();
    t = g_tm;
    portEXIT_CRITICAL(&g_mux);
    Serial.printf(
        "ARM TIMING,ticks=%lu,active=%lu,cmax_us=%lu,cmean_us=%lu,jitter_max_us=%lu,late=%lu,dropped=%lu,sat=%lu\n",
        (unsigned long)t.ticks, (unsigned long)t.active, (unsigned long)t.cmaxUs,
        (unsigned long)(t.active ? t.csumUs / t.active : 0), (unsigned long)t.jmaxUs, (unsigned long)t.late,
        (unsigned long)t.dropped, (unsigned long)t.sat);
  } else if (!strcmp(sub, "OFF")) {
    g_outReq = false;  // task tat xung o nhip ke tiep; chot toi khi reset
    Serial.println("OK ARM OFF (ngung xung servo; dat tay ve Home roi nhan EN de bat lai)");
  } else if (!strcmp(sub, "STATUS")) {
    motion::Command c;
    float us[NCH];
    portENTER_CRITICAL(&g_mux);
    c = g_last;
    for (uint8_t i = 0; i < NCH; i++) us[i] = g_lastUs[i];
    portEXIT_CRITICAL(&g_mux);
    Serial.printf("ARM STATUS,busy=%d,output=%d,phase=%s,servo=%.2f/%.2f/%.2f,grip=%.2f,us=%.0f/%.0f/%.0f/%.0f\n",
                  busy() ? 1 : 0, (ARM_OUTPUT_ENABLED && g_outReq) ? 1 : 0, motion::phaseName(c.phase),
                  c.servoDeg[0], c.servoDeg[1], c.servoDeg[2], c.gripDeg, us[0], us[1], us[2], us[3]);
  } else {
    Serial.println("ERR ARM_UNKNOWN_CMD");
  }
  return true;
}
