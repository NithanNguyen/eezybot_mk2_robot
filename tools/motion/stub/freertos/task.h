#pragma once
#include "FreeRTOS.h"
typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char* name, uint32_t stack, void* arg, UBaseType_t prio,
                                   TaskHandle_t* handle, BaseType_t core);
TickType_t xTaskGetTickCount();
void vTaskDelayUntil(TickType_t* last, TickType_t period);
