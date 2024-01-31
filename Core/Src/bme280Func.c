#include "bme280Func.h"
#include "application.h"

#define BME280_F_SPI hspi1
#define BME280_F_CS_PORT BME280_CS_GPIO_Port
#define BME280_F_CS_PIN BME280_CS_Pin
#define BME280_INTF_BUSY 1
#define BME280_INTF_FREE 0

struct bme280_dev gxDev;

int32_t icBmeSetup(struct bme280_dev *xDev)
{
    struct bme280_settings xSettings = {0};

    int32_t ilRslt = 0;

    xDev->intf = BME280_SPI_INTF;
    xDev->read = icBmeSpiRead;
    xDev->write = icBmeSpiWrite;
    xDev->delay_us = vBmeDelayUs;
    xDev->intf_ptr = &BME280_F_SPI;

    ilRslt = bme280_init(xDev);
    if (ilRslt != BME280_OK)
    {
        return ilRslt;
    }

    /* Get the current sensor settings */
    ilRslt = bme280_get_sensor_settings(&xSettings, xDev);
    if (ilRslt != BME280_OK)
    {
        return ilRslt;
    }

    /* Recommended mode of operation: Indoor navigation */
    xSettings.filter = BME280_FILTER_COEFF_16;
    xSettings.osr_h = BME280_OVERSAMPLING_16X;
    xSettings.osr_p = BME280_OVERSAMPLING_8X;
    xSettings.osr_t = BME280_OVERSAMPLING_16X;
    xSettings.standby_time = BME280_STANDBY_TIME_1000_MS;

    /* Set the sensor settings */
    ilRslt = bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &xSettings, xDev);
    if (ilRslt != BME280_OK)
    {
        return ilRslt;
    }

    ilRslt = bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, xDev);
    if (ilRslt != BME280_OK)
    {
        return ilRslt;
    }

    xDev->delay_us(100000, NULL);

    return ilRslt;
}

int32_t ilBmeGetMeasurement(struct bme280_dev *xDev, Meas_t *xMeasurement)
{
    int32_t ilRslt;
    struct bme280_data xCompData;

    ilRslt = bme280_get_sensor_data(BME280_ALL, &xCompData, xDev);
    if (ilRslt != BME280_OK)
    {
        return ilRslt;
    }

    xMeasurement->temp = 0.01f * xCompData.temperature;          // C
    xMeasurement->press = 0.01f * 0.750062 * xCompData.pressure; // mm
    xMeasurement->hum = 1.0f / 1024.0f * xCompData.humidity;     // %

    return ilRslt;
}

int32_t ilStreamSensorDataForcedMode(struct bme280_dev *xDev, Meas_t *xMeasurement)
{
    int32_t ilRslt;

    /* Continuously stream sensor data */

    ilRslt = bme280_set_sensor_mode(BME280_POWERMODE_FORCED, xDev);
    if (ilRslt != BME280_OK)
    {
        return ilRslt;
    }

    /* Wait for the measurement to complete and print data @25Hz */
    //	xDev->delay_us(40000, xDev->intf_ptr);
    uint8_t ucStatus_reg = BME280_STATUS_MEAS_DONE;
    while (ucStatus_reg & BME280_STATUS_MEAS_DONE)
    {
        xDev->delay_us(20000, NULL);
        bme280_get_regs(BME280_REG_STATUS, &ucStatus_reg, 1, xDev);
    }

    ilRslt = ilBmeGetMeasurement(xDev, xMeasurement);

    return ilRslt;
}

void vBmeDelayUs(uint32_t ulPeriod, void *pvIntf)
{
    vTaskDelay(ulPeriod / 1000);
}

int8_t icBmeSpiRead(uint8_t ulRegAddr, uint8_t *pucRegData, uint32_t ulLen, void *pvIntf)
{
    /*
     * The parameter intf_ptr can be used as a variable to select which Chip Select pin has
     * to be set low to activate the relevant device on the SPI bus
     */

    /*
     * Data on the bus should be like
     * |----------------+---------------------+-------------|
     * | MOSI           | MISO                | Chip Select |
     * |----------------+---------------------|-------------|
     * | (don't care)   | (don't care)        | HIGH        |
     * | (reg_addr)     | (don't care)        | LOW         |
     * | (don't care)   | (pucRegData[0])     | LOW         |
     * | (....)         | (....)              | LOW         |
     * | (don't care)   |(pucRegData[len - 1])| LOW         |
     * | (don't care)   | (don't care)        | HIGH        |
     * |----------------+---------------------|-------------|
     */
    int8_t icRslt = 0; /* Return 0 for Success, non-zero for failure */

    uint8_t pucTxArray[SPI_BUFFER_LEN] = {0};
    uint8_t pucRxArray[SPI_BUFFER_LEN] = {0};

    /*	For the SPI mode only 7 bits of register addresses are used.
    The MSB of register address is declared the bit what functionality it is
    read/write (read as 1/write as BME280_INIT_VALUE)*/
    pucTxArray[0] = ulRegAddr | SPI_READ; /*read routine is initiated register address is mask with 0x80*/
    HAL_GPIO_WritePin(BME280_F_CS_PORT, BME280_F_CS_PIN, GPIO_PIN_RESET);
    icRslt = HAL_SPI_TransmitReceive_IT(&BME280_F_SPI, pucTxArray, pucRxArray, ulLen + 1);
    xSemaphoreTake(xBme280IntfSemHandle, portMAX_DELAY);
    HAL_GPIO_WritePin(BME280_F_CS_PORT, BME280_F_CS_PIN, GPIO_PIN_SET);

    for (uint8_t i = 0; i < ulLen; i++)
    {
        pucRegData[i] = pucRxArray[i + 1];
    }

    return icRslt;
}

int8_t icBmeSpiWrite(uint8_t ulRegAddr, const uint8_t *pucRegData, uint32_t ulLen, void *pvIntf)
{
    int8_t icRslt = 0; /* Return 0 for Success, non-zero for failure */
    uint8_t pucArray[SPI_BUFFER_LEN * 2];

    for (uint8_t i = 0; i < ulLen; i++)
    {
        pucArray[i * 2] = ulRegAddr & SPI_WRITE;
        ulRegAddr++;
        pucArray[i * 2 + 1] = pucRegData[i];
    }

    HAL_GPIO_WritePin(BME280_F_CS_PORT, BME280_F_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit_IT(&BME280_F_SPI, pucArray, ulLen * 2);
    xSemaphoreTake(xBme280IntfSemHandle, portMAX_DELAY);
    HAL_GPIO_WritePin(BME280_F_CS_PORT, BME280_F_CS_PIN, GPIO_PIN_SET);

    return icRslt;
}
