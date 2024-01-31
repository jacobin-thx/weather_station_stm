#ifndef CRC_H
#define CRC_H

#include <stddef.h>
#include <stdint.h>

uint8_t ucCrcCalc(uint8_t *pucTxData, uint8_t len);
uint8_t ucCrcCheck(uint8_t *pucRxData, uint8_t len);

#endif /* crc.h */