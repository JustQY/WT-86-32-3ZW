/**************************************************************************/
/*!
    @file     GSL2038.h

    @thanks to Skallwar for the source code this lib is bassed on : https://github.com/Skallwar/GSL1680
*/
#include <stdint.h>
#include "stdbool.h"

#ifndef GSL2038_H
#define GSL2038_H

#define I2CADDR 0x40
#define DATA_REG 0x80
#define STATUS_REG 0xE0
#define PAGE_REG 0xF0

typedef struct
{
   uint8_t FingerID;
   uint16_t X;
   uint16_t Y;
}TouchPointType,*PTouchPoint;

/*typedef struct{
    uint8_t Count;
    PointType Points[5];
}TouchPointType,*PTouchPoint;
*/

void GSL2038Init();
bool GSL2308GetPoint(uint16_t *x,uint16_t *y);
void Loop();

#endif
