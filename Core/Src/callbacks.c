#include "application.h"
#include "comDefs.h"
#include "stm32g4xx_it.h"

#define TICK_FOR_1MS_TIM 5

extern eAutoMode geHeaterAutoMode;
extern eAutoMode geHumidifAutoMode;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xUartTxSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xUartRxSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        // bme280_set_it_flag();
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xBme280IntfSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        // bme280_set_it_flag();
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(xBme280IntfSemHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) // колбек по захвату
{
    if (htim->Instance == TIM1)
    {
        uint16_t but_fall;
        uint16_t but_rise;
        uint16_t delta;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        uint8_t command[5];
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) // FALLING с HIGH на LOW
        {
            but_fall = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
            but_rise = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
            delta = but_rise - but_fall;

            if (delta > 1000 * TICK_FOR_1MS_TIM)
            {
                // long press
                geHumidifAutoMode = AUTOMODE_ON;
                command[0] = CTRL_HUMIDIF_AUTO_MODE;
                xQueueSendToBackFromISR(xUnitControlQueueHandle, command, &xHigherPriorityTaskWoken);
            }
            else if (delta > 50 * TICK_FOR_1MS_TIM)
            {
                // short press
                geHumidifAutoMode = AUTOMODE_OFF;
                command[0] = CTRL_HUMIDIF_CHNG_STATE;
                xQueueSendToBackFromISR(xUnitControlQueueHandle, command, &xHigherPriorityTaskWoken);
            }
        }
        else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // FALLING с HIGH на LOW
        {
            but_fall = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_3);
            but_rise = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_4);
            delta = but_rise - but_fall;

            if (delta > 1000 * TICK_FOR_1MS_TIM)
            {
                // long press
                geHeaterAutoMode = AUTOMODE_ON;
                command[0] = CTRL_HEATER_AUTO_MODE;
                xQueueSendToBackFromISR(xUnitControlQueueHandle, command, &xHigherPriorityTaskWoken);
                // xSemaphoreGive(xMeasPeriodSemHandle);
            }
            else if (delta > 50 * TICK_FOR_1MS_TIM)
            {
                // short press
                geHeaterAutoMode = AUTOMODE_OFF;
                command[0] = CTRL_HEATER_CHNG_STATE;
                xQueueSendToBackFromISR(xUnitControlQueueHandle, command, &xHigherPriorityTaskWoken);
            }
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}