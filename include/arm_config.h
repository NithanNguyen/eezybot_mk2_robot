// arm_config.h - SINH TU DONG boi eezybot_mk2_sim/tools/export_arm_config.py tu params.py. KHONG SUA TAY:
// sua eezybot_mk2_sim/params.py roi chay lai `python tools/export_arm_config.py`.
// Nguon: params.py + sort_scene.py, SHA-256 d1bb946a8cb41a44
// Don vi: mm, do, giay. Goc khop: theta1 = de, theta2 = canh tay chinh so voi phuong ngang,
// phi = tay ngang so voi phuong ngang (servo khuyu dat phi tuyet doi), phi - theta2 = goc khuyu tuong doi.
#pragma once

namespace armcfg {

struct Point3 { float x, y, z; };

// ---- Hinh hoc co cau [DO-STL] (params.py muc 1)
constexpr float L_MAIN_ARM = 135.0f;  // L2: vai O -> khuyu E
constexpr float L_HORIZ_ARM = 147.0f;  // L3: khuyu E -> co tay W
constexpr float L_HORIZ_TAIL = 57.0f;
constexpr float L_CRANK = 57.0f;
constexpr float L_LINK_A = 135.0f;
constexpr float L_LINK_B = 147.0f;
constexpr float L_ELBOW_TRI = 43.0f;
constexpr float SHOULDER_HEIGHT = 94.25f;  // H: truc vai so voi mat ban (suy tu STL, nen do lai)
constexpr float SHOULDER_OFFSET_X = 0.0f;  // truc vai so voi truc de theo phuong ngang
constexpr float BASE_GEAR_RATIO = 2.0f;  // goc servo de = ti so * theta1

// ---- Dau cong tac so voi tam co tay W: (DX, DZ) trong mat phang canh tay, DY lech ngang (params.py muc 2)
constexpr float TOOL_GRIPPER_DX = 132.69f;  // TCP gap vat
constexpr float TOOL_GRIPPER_DZ = 0.5f;
constexpr float TOOL_CAP_DX = 110.0f;  // diem 'cap' = tam nap khi kep (bai phan loai)
constexpr float TOOL_CAP_DZ = 0.5f;
constexpr float TOOL_DY = 0.48f;

// ---- Gioi han khop (do), tu kiem tra va cham STL (params.py muc 3). IK phai kiem TAT CA cac dieu kien
// duoi day giong mk2_kinematics.within_limits().
constexpr float THETA1_MIN = -45.0f;
constexpr float THETA1_MAX = 45.0f;
constexpr float THETA2_MIN = 42.5f;
constexpr float THETA2_MAX = 125.0f;
constexpr float PHI_MIN = -65.0f;
constexpr float PHI_MAX = 20.0f;
constexpr float ELBOW_MIN = -145.0f;  // phi - theta2
constexpr float ELBOW_MAX = -60.0f;
// Tam lenh actuator (do): de = BASE_GEAR_RATIO * theta1, vai = theta2 - 90, khuyu = phi
constexpr float CTRL_BASE_MIN = -90.0f;
constexpr float CTRL_BASE_MAX = 90.0f;
constexpr float CTRL_SHOULDER_MIN = -57.0f;
constexpr float CTRL_SHOULDER_MAX = 40.0f;
constexpr float CTRL_ELBOW_MIN = -62.5f;
constexpr float CTRL_ELBOW_MAX = 17.5f;

// ---- Anh xa goc khop -> goc servo (do) [GIA-DINH, phai hieu chuan] (params.py muc 4):
//   servo[0] = OFFSET[0] + SIGN[0] * BASE_GEAR_RATIO * theta1
//   servo[1] = OFFSET[1] + SIGN[1] * (theta2 - 90)
//   servo[2] = OFFSET[2] + SIGN[2] * phi
constexpr float SERVO_OFFSET_DEG[3] = {90.0f, 90.0f, 90.0f};
constexpr float SERVO_SIGN[3] = {1.0f, 1.0f, 1.0f};
constexpr float SERVO_MIN_DEG = 0.0f;
constexpr float SERVO_MAX_DEG = 180.0f;
// Home: theta1 = 0, theta2 = 90 (tay chinh dung), phi = 0 (tay ngang nam ngang)
constexpr float HOME_THETA1 = 0.0f;
constexpr float HOME_THETA2 = 90.0f;
constexpr float HOME_PHI = 0.0f;

// ---- Anh xa goc khop -> do rong xung (us) [DO-THAT] (params.py muc 4, tools/servo_cal cua firmware):
//   us[i] = SERVO_PULSE_HOME_US[i] + SERVO_PULSE_PER_DEG[i] * (q_i - HOME_i), q = (theta1, theta2, phi) do
constexpr float SERVO_PULSE_HOME_US[3] = {1500.0f, 1500.0f, 1490.1f};
constexpr float SERVO_PULSE_PER_DEG[3] = {-27.4924f, -12.6536f, 11.639f};
constexpr unsigned MG996R_MIN_US = 500, MG996R_MAX_US = 2500;  // gioi han phan mem [CHON]
// Kep: us = GRIP_PULSE_CLOSED_US + GRIP_PULSE_PER_DEG * (goc truc SG90 / GRIPPER_GEAR_RATIO)  [GIA-DINH: chua do, SG90 hong]
constexpr float GRIP_PULSE_CLOSED_US = 1223.7f;
constexpr float GRIP_PULSE_PER_DEG = 7.4491f;
constexpr bool GRIP_PULSE_MEASURED = false;

// ---- Quy dao double-S (params.py muc 8). Gioi han cua DAU CONG TAC, dung cho ca duong thang (ha/nhac)
// va chuyen dong khop: q(t) = q0 + sigma(t) (q1 - q0) — gian thoi gian de dau cong tac khong vuot.
constexpr float V_MAX = 150.0f;  // mm/s
constexpr float A_MAX = 1500.0f;  // mm/s^2
constexpr float J_MAX = 30000.0f;  // mm/s^3
constexpr float TRAJ_R_REF = 280.0f;  // mm, ban kinh quy doi sang gioi han khop [CHON]
constexpr float JOINT_V_MAX = V_MAX / TRAJ_R_REF;  // rad/s
constexpr float JOINT_A_MAX = A_MAX / TRAJ_R_REF;  // rad/s^2
constexpr float JOINT_J_MAX = J_MAX / TRAJ_R_REF;  // rad/s^3
constexpr float CONTROL_PERIOD_S = 0.02f;  // = chu ky xung servo 20 ms

// ---- Kep SG90 (params.py muc 5 va 8). Goc kep tinh tu luc dong (0 = dong).
constexpr unsigned SG90_MIN_US = 500, SG90_MAX_US = 2400;
constexpr float GRIPPER_GEAR_RATIO = 1.0f;
constexpr float GRIP_OPEN_DEG = 40.0f;  // mo khi ha xuong
constexpr float GRIP_HOLD_DEG = 15.0f;  // lenh kep nap
constexpr float GRIP_MOVE_S = 0.5f;  // cho sau moi lenh dong/mo
constexpr float DROP_WAIT_S = 0.6f;  // cho nap roi sau khi mo kep

// ---- Chu trinh phan loai: diem 'cap' (tam nap khi kep, mm, he the gioi; dung TOOL_CAP_*) — sort_scene.task_points()
constexpr float DROP_HEIGHT_ABOVE_BIN = 50.0f;  // day nap cach mat tren hop / mat ban khi tha
constexpr float PRE_GRASP_LIFT = 40.0f;
constexpr Point3 P_GRASP = {279.0f, 0.0f, 59.0f};  // diem kep tai tram
constexpr Point3 P_PRE_GRASP = {279.0f, 0.0f, 99.0f};  // phia tren diem kep
constexpr Point3 P_DROP_RED = {280.592231f, -162.0f, 116.0f};  // ngan 1
constexpr Point3 P_DROP_GREEN = {230.362757f, -133.0f, 116.0f};  // ngan 2
constexpr Point3 P_DROP_WHITE = {180.133284f, -104.0f, 116.0f};  // ngan 3
constexpr Point3 P_REJECT_PRE = {246.81724f, 142.5f, 99.0f};  // phia tren vung vut bo, roi ha thang
constexpr Point3 P_DROP_REJECT = {246.81724f, 142.5f, 66.0f};

}  // namespace armcfg
