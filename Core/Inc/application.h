#ifndef APPLICATION_H
#define APPLICATION_H

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"


extern SemaphoreHandle_t xBme280IntfSemHandle;
extern SemaphoreHandle_t xUartTxSemHandle;
extern SemaphoreHandle_t xUartRxSemHandle;
extern SemaphoreHandle_t xMeasPeriodSemHandle;
extern QueueHandle_t xUnitControlQueueHandle;

typedef enum AutoMode {
    AUTOMODE_ON = 0,
    AUTOMODE_OFF = 1
} eAutoMode;

typedef enum Flag {
    FLAG_FALSE = 0,
    FLAG_TRUE = 1
} eFlag;



#endif