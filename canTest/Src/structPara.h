#ifndef __STRUCTPARA_H
#define __STRUCTPARA_H

#include "stdint.h"

typedef struct{
    uint8_t update;
    uint16_t channel[18];
}sBusDataStruct;

extern sBusDataStruct sBusData;

typedef struct{
    uint8_t update;
    float temperature;
    float speed;
    float rtSpeed; 
    float pressureDiff;
}airSpeedStruct;

extern airSpeedStruct airSpeed;

#endif