#ifndef __STRUCTPARA_H
#define __STRUCTPARA_H

typedef struct{
    uint8_t updata;
    uint16_t channel[18];
}sbusStruct;

extern sbusDataStruct sBusData;

typedef struct{
    uint8_t update;
    float temperature;
    float speed;
    float rtSpeed; 
    float pressureDiff;
}airSpeedStruct;

extern airSpeedStruct airSpeed;

#endif