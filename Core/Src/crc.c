#include "crc.h"

uint8_t ucCrcCalc(uint8_t *pucTxData, uint8_t ucLen)
{
    uint8_t ucCrc = 0;
    for (size_t i = 0; i < ucLen - 1; i++)
    {
        ucCrc -= pucTxData[i];
    }
    pucTxData[ucLen - 1] = ucCrc;
    return ucLen;
}
uint8_t ucCrcCheck(uint8_t *pucRxData, uint8_t ucLen)
{
    uint8_t ucSum = 0;
    for (size_t i = 0; i < ucLen; i++)
    {
        ucSum += pucRxData[i];
    }
    return ucSum;
}