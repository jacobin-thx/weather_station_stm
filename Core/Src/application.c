#include "application.h"

#include "bme280Func.h"
#include "comDefs.h"
#include "timers.h"
#include "crc.h"

#define HUM_THRESHOLD 20.0f
#define HUM_GISTERESIS 1.0f
#define TEMP_THRESHOLD 25.0f
#define TEMP_GISTERESIS 0.5f

uint8_t ucCtrlHumidif(float fHum, float hum_thres, uint8_t ulHumididOldState);
uint8_t ucCtrlHeater(float fTemp, float fTempThres, uint8_t ulHeaterOldState);

void vMeasurementTask(void *argument);
void vUnitControlTask(void *argument);
void vTxEspTask(void *argument);
void vRxEspTask(void *argument);
static void vMeasTimerCallback(TimerHandle_t xTimer);

SemaphoreHandle_t xBme280IntfSemHandle;
SemaphoreHandle_t xUnitControlQueueHandle;
SemaphoreHandle_t xUartRxSemHandle;
SemaphoreHandle_t xUartTxSemHandle;
SemaphoreHandle_t xMeasPeriodSemHandle;
QueueHandle_t xTxEspQueueHandle;
QueueHandle_t xUnitControlQueueHandle;
TimerHandle_t xMeasTimerHandle;

uint32_t gulMeasPeriod = 3; // Seconds
eAutoMode geHeaterAutoMode = AUTOMODE_ON;
eAutoMode geHumidifAutoMode = AUTOMODE_ON;
eFlag geFlagNewHeaterThres = FLAG_FALSE;
eFlag geFlagNewHumidifThres = FLAG_FALSE;

int main(void)
{
    vStmInit();
    xTaskCreate(vMeasurementTask, "MeasurementTask", 192, NULL, 24, NULL);
    xTaskCreate(vUnitControlTask, "UnitControlTask", 64, NULL, 40, NULL);
    xTaskCreate(vTxEspTask, "TxEspTask", 64, NULL, 16, NULL);
    xTaskCreate(vRxEspTask, "RxEspTask", 64, NULL, 8, NULL);

    xBme280IntfSemHandle = xSemaphoreCreateBinary();
    xUartTxSemHandle = xSemaphoreCreateBinary();
    xUartRxSemHandle = xSemaphoreCreateBinary();
    xMeasPeriodSemHandle = xSemaphoreCreateBinary();
    xMeasTimerHandle = xTimerCreate("measTimer", 1, pdTRUE, 0, vMeasTimerCallback);
    xTxEspQueueHandle = xQueueCreate(16, 5);
    xUnitControlQueueHandle = xQueueCreate(16, 5);

    vTaskStartScheduler();
    while (1)
    {
        __NOP();
    }
}

void vMeasurementTask(void *argument)
{
    TickType_t xLastWakeTime;        /* time for vTaskDelayUntil */
    Meas_t xMeasurement;             /* structure for last measurements */
    Meas_t xMeasurementOld = {.temp = -500, .hum = -1, .press = -1};
    uint8_t pucTxBuff[UART_DATA_SIZE];         /* buffer to send to the control task */
    int32_t lRslt = 0;

    xLastWakeTime = xTaskGetTickCount();

    icBmeSetup(&gxDev);

    struct bme280_settings xSettings = {0};
    lRslt = bme280_get_sensor_settings(&xSettings, &gxDev);
    if (lRslt != BME280_OK)
    {
        __NOP();
    }

    pucTxBuff[0] = COM_START;
    xQueueSendToBack(xTxEspQueueHandle, &pucTxBuff, 0);

    xTimerChangePeriod(xMeasTimerHandle, pdMS_TO_TICKS(gulMeasPeriod * 1000), portMAX_DELAY);

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);

    // Delay until threshold will be rxed
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Infinite loop */
    for (;;)
    {
        lRslt = ilBmeGetMeasurement(&gxDev, &xMeasurement);

        if (lRslt == BME280_OK)
        {
            if ((geHumidifAutoMode == AUTOMODE_ON && xMeasurementOld.hum != xMeasurement.hum) || geFlagNewHumidifThres == FLAG_TRUE)
            {
                geFlagNewHumidifThres = FLAG_FALSE;
                xMeasurementOld.hum = xMeasurement.hum;
                pucTxBuff[0] = CTRL_HUMIDIF_HUM;
                *(float *)(pucTxBuff + 1) = xMeasurement.hum;
                xQueueSendToBack(xUnitControlQueueHandle, pucTxBuff, 0);
            }
            if (((geHeaterAutoMode == AUTOMODE_ON) && (xMeasurementOld.temp != xMeasurement.temp)) || geFlagNewHeaterThres == FLAG_TRUE)
            {
                geFlagNewHeaterThres = FLAG_FALSE;
                xMeasurementOld.temp = xMeasurement.temp;
                pucTxBuff[0] = CTRL_HEATER_TEMP;
                *(float *)(pucTxBuff + 1) = xMeasurement.temp;
                xQueueSendToBack(xUnitControlQueueHandle, pucTxBuff, 0);
            }

            if (xSemaphoreTake(xMeasPeriodSemHandle, 0) == pdPASS)
            {
                pucTxBuff[0] = COM_DATA_TEMP;
                *(float *)(pucTxBuff + 1) = xMeasurement.temp;
                xQueueSendToBack(xTxEspQueueHandle, pucTxBuff, 0);

                pucTxBuff[0] = COM_DATA_HUM;
                *(float *)(pucTxBuff + 1) = xMeasurement.hum;
                xQueueSendToBack(xTxEspQueueHandle, pucTxBuff, 0);

                pucTxBuff[0] = COM_DATA_PRESS;
                *(float *)(pucTxBuff + 1) = xMeasurement.press;
                xQueueSendToBack(xTxEspQueueHandle, pucTxBuff, 0);
            }
        }

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void vUnitControlTask(void *argument)
{
    uint8_t pucBuffer[5];
    float fHum = 100, fTemp = 100;
    float fHumThres = HUM_THRESHOLD, fTempThres = TEMP_THRESHOLD;
    uint32_t ulHeaterState = 0xFF, ulHumidifState = 0xFF;
    uint32_t ulHeaterOldState = 0xFF, ulHumididOldState = 0xFF;
    /* Infinite loop */
    for (;;)
    {
        xQueueReceive(xUnitControlQueueHandle, pucBuffer, portMAX_DELAY);

        switch (pucBuffer[0])
        {
        case CTRL_HEATER_TEMP:
            fTemp = *(float *)(pucBuffer + 1);
            ulHeaterState = ucCtrlHeater(fTemp, fTempThres, ulHeaterOldState);
            break;
        case CTRL_HEATER_THRES:
            fTempThres = *(float *)(pucBuffer + 1);
            break;
        case CTRL_HEATER_CHNG_STATE:
            HAL_GPIO_TogglePin(HEATER_CTRL_GPIO_Port, HEATER_CTRL_Pin);
            if (ulHeaterState == 0 || ulHeaterState == 2)
            {
                ulHeaterState = 3;
            }
            else
            {
                ulHeaterState = 2;
            }
            break;
        case CTRL_HEATER_OFF:
            HAL_GPIO_WritePin(HEATER_CTRL_GPIO_Port, HEATER_CTRL_Pin, 0);
            ulHeaterState = 2;
            break;
        case CTRL_HEATER_ON:
            HAL_GPIO_WritePin(HEATER_CTRL_GPIO_Port, HEATER_CTRL_Pin, 1);
            ulHeaterState = 3;
            break;
        case CTRL_HEATER_AUTO_MODE:
            ulHeaterState = ucCtrlHeater(fTemp, fTempThres, ulHeaterOldState);
            break;

        case CTRL_HUMIDIF_HUM:
            fHum = *(float *)(pucBuffer + 1);
            ulHumidifState = ucCtrlHumidif(fHum, fHumThres, ulHumididOldState);
            break;
        case CTRL_HUMIDIF_THRES:
            fHumThres = *(float *)(pucBuffer + 1);
            break;
        case CTRL_HUMIDIF_CHNG_STATE:
            HAL_GPIO_TogglePin(HUMIDIF_CTRL_GPIO_Port, HUMIDIF_CTRL_Pin);
            if (ulHumidifState == 0 || ulHumidifState == 2)
            {
                ulHumidifState = 3;
            }
            else
            {
                ulHumidifState = 2;
            }
            break;
        case CTRL_HUMIDIF_OFF:
            HAL_GPIO_WritePin(HUMIDIF_CTRL_GPIO_Port, HUMIDIF_CTRL_Pin, 0);
            ulHumidifState = 2;
            break;
        case CTRL_HUMIDIF_ON:
            HAL_GPIO_WritePin(HUMIDIF_CTRL_GPIO_Port, HUMIDIF_CTRL_Pin, 1);
            ulHumidifState = 3;
            break;
        case CTRL_HUMIDIF_AUTO_MODE:
            ulHumidifState = ucCtrlHumidif(fHum, fHumThres, ulHumididOldState);
            break;

        default:
            __NOP();
        }
        if (ulHeaterState != ulHeaterOldState)
        {
            ulHeaterOldState = ulHeaterState;
            pucBuffer[0] = COM_HEATER_STATE;
            *(uint32_t *)(pucBuffer + 1) = ulHeaterState;
            xQueueSendToBack(xTxEspQueueHandle, pucBuffer, portMAX_DELAY);
        }
        if (ulHumidifState != ulHumididOldState)
        {
            ulHumididOldState = ulHumidifState;
            pucBuffer[0] = COM_HUMIDIF_STATE;
            *(uint32_t *)(pucBuffer + 1) = ulHumidifState;
            xQueueSendToBack(xTxEspQueueHandle, pucBuffer, portMAX_DELAY);
        }
    }
}

void vTxEspTask(void *argument)
{
    uint8_t pucTxBuff[UART_DATA_SIZE];
    /* Infinite loop */
    for (;;)
    {
        xQueueReceive(xTxEspQueueHandle, pucTxBuff, portMAX_DELAY);
        ucCrcCalc(pucTxBuff, UART_DATA_SIZE);
        HAL_UART_Transmit_IT(&huart3, pucTxBuff, UART_DATA_SIZE);
        xSemaphoreTake(xUartTxSemHandle, portMAX_DELAY);
    }
}

void vRxEspTask(void *argument)
{
    uint8_t rxData[UART_DATA_SIZE];
    uint8_t pucBuffer[5];
    /* Infinite loop */
    for (;;)
    {
        HAL_UART_Receive_IT(&huart3, rxData, UART_DATA_SIZE);
        xSemaphoreTake(xUartRxSemHandle, portMAX_DELAY);
        if (ucCrcCheck(rxData, UART_DATA_SIZE))
        {
            __NOP();
        }
        else
        {
            switch (rxData[0])
            {
            case COM_CHNG_PERIOD:
                gulMeasPeriod = *(uint32_t *)(rxData + 1);
                xTimerChangePeriod(xMeasTimerHandle, pdMS_TO_TICKS(gulMeasPeriod * 1000), portMAX_DELAY);
                xSemaphoreGive(xMeasPeriodSemHandle);
                break;

            case COM_MEAS_REQUEST:
                xTimerStart(xMeasTimerHandle, portMAX_DELAY);
                xSemaphoreGive(xMeasPeriodSemHandle);
                break;

            case COM_HEATER_COMMAND:
                if (rxData[1] == SET_MODE_OFF)
                {
                    geHeaterAutoMode = AUTOMODE_OFF;
                    pucBuffer[0] = CTRL_HEATER_OFF;
                }
                else if (rxData[1] == SET_MODE_ON)
                {
                    geHeaterAutoMode = AUTOMODE_OFF;
                    pucBuffer[0] = CTRL_HEATER_ON;
                }
                else if (rxData[1] == SET_MODE_AUTO)
                {
                    geHeaterAutoMode = AUTOMODE_ON;
                    pucBuffer[0] = CTRL_HEATER_AUTO_MODE;
                }
                xQueueSendToBack(xUnitControlQueueHandle, pucBuffer, portMAX_DELAY);
                break;
            case COM_HUMIDIF_COMMAND:
                if (rxData[1] == SET_MODE_OFF)
                {
                    geHumidifAutoMode = AUTOMODE_OFF;
                    pucBuffer[0] = CTRL_HUMIDIF_OFF;
                }
                else if (rxData[1] == SET_MODE_ON)
                {
                    geHumidifAutoMode = AUTOMODE_OFF;
                    pucBuffer[0] = CTRL_HUMIDIF_ON;
                }
                else if (rxData[1] == SET_MODE_AUTO)
                {
                    geHumidifAutoMode = AUTOMODE_ON;
                    pucBuffer[0] = CTRL_HUMIDIF_AUTO_MODE;
                }
                xQueueSendToBack(xUnitControlQueueHandle, pucBuffer, portMAX_DELAY);
                break;

            case COM_HEATER_THRESHOLD:
                pucBuffer[0] = CTRL_HEATER_THRES;
                *(uint32_t *)(pucBuffer + 1) = *(uint32_t *)(rxData + 1);
                xQueueSendToBack(xUnitControlQueueHandle, pucBuffer, portMAX_DELAY);
                geFlagNewHeaterThres = FLAG_TRUE;
                break;
            case COM_HUMIDIF_THRESHOLD:
                pucBuffer[0] = CTRL_HUMIDIF_THRES;
                *(uint32_t *)(pucBuffer + 1) = *(uint32_t *)(rxData + 1);
                xQueueSendToBack(xUnitControlQueueHandle, pucBuffer, portMAX_DELAY);
                geFlagNewHumidifThres = FLAG_TRUE;
                break;

            default:
                break;
            }
        }
    }
}

static void vMeasTimerCallback(TimerHandle_t xTimer)
{
    xSemaphoreGive(xMeasPeriodSemHandle);
}

uint8_t ucCtrlHeater(float fTemp, float fTempThres, uint8_t ulHeaterOldState)
{
    uint8_t ulHeaterState = ulHeaterOldState;
    if (fTemp < fTempThres - TEMP_GISTERESIS)
    {
        ulHeaterState = 1;
        HAL_GPIO_WritePin(HEATER_CTRL_GPIO_Port, HEATER_CTRL_Pin, 1);
    }
    else if (fTemp > fTempThres + TEMP_GISTERESIS)
    {
        ulHeaterState = 0;
        HAL_GPIO_WritePin(HEATER_CTRL_GPIO_Port, HEATER_CTRL_Pin, 0);
    }
    return ulHeaterState;
}

uint8_t ucCtrlHumidif(float fHum, float fHumThres, uint8_t ulHumididOldState)
{
    uint8_t ulHumidifState = ulHumididOldState;
    if (fHum < fHumThres - HUM_GISTERESIS)
    {
        ulHumidifState = 1;
        HAL_GPIO_WritePin(HUMIDIF_CTRL_GPIO_Port, HUMIDIF_CTRL_Pin, 1);
    }
    else if (fHum > fHumThres + HUM_GISTERESIS)
    {
        ulHumidifState = 0;
        HAL_GPIO_WritePin(HUMIDIF_CTRL_GPIO_Port, HUMIDIF_CTRL_Pin, 0);
    }
    return ulHumidifState;
}
