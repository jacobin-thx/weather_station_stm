/*
 * bme280-func.h
 *
 *  Created on: Dec 26, 2023
 *      Author: paul
 */

#ifndef INC_BME280_FUNC_H_
#define INC_BME280_FUNC_H_

#define BME280_32BIT_ENABLE
#include "bme280.h"
#include "main.h"

#define SPI_READ 0x80
#define SPI_WRITE 0x7F
#define SPI_BUFFER_LEN 28

extern struct bme280_dev gxDev;

typedef struct Meas_t {
  float temp;
  float press;
  float hum;
} Meas_t;

int32_t icBmeSetup(struct bme280_dev *dev);
int32_t ilStreamSensorDataForcedMode(struct bme280_dev *dev, Meas_t *measurement);
int32_t ilBmeGetMeasurement(struct bme280_dev *dev, Meas_t *measurement);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read functions of the user
 *
 * @param[in] reg_addr       : Register address from which data is read.
 * @param[out] reg_data      : Pointer to data buffer where read data is stored.
 * @param[in] len            : Number of bytes of data to be read.
 * @param[in, out] intf_ptr  : Void pointer that can enable the linking of descriptors
 *                             for interface related call backs.
 *
 * @retval   0 -> Success.
 * @retval Non zero value -> Fail.
 *
 */
int8_t icBmeSpiRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific write functions of the user
 *
 * @param[in] reg_addr      : Register address to which the data is written.
 * @param[in] reg_data      : Pointer to data buffer in which data to be written
 *                            is stored.
 * @param[in] len           : Number of bytes of data to be written.
 * @param[in, out] intf_ptr : Void pointer that can enable the linking of descriptors
 *                            for interface related call backs
 *
 * @retval   0   -> Success.
 * @retval Non zero value -> Fail.
 *
 */
int8_t icBmeSpiWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr);

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs.
 *
 * @param[in] period              : Delay in microseconds.
 * @param[in, out] intf_ptr       : Void pointer that can enable the linking of descriptors
 *                                  for interface related call backs
 *  @return void.
 *
 */
void vBmeDelayUs(uint32_t period, void *intf_ptr);

#endif /* INC_BME280_FUNC_H_ */
