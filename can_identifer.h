#ifndef __CAN_IDENTIFER_H
#define __CAN_IDENTIFER_H

#define canMegFilter 0x01f8 // 0b00111111000 

#define canIdOffset (9) 

/*遵循 越小优先级越高*/ 

#define canMaster (0x00 << canIdOffset)
#define canSlave (0x01 << canIdOffset)

#define canMegOffset (3)

/*最高级消息(16组)*/
/*0x00 ~ 0x0F(0b00 0000 ~ 0b00 1111)*/
#define canHigMegBase 0x00

#define canPWMOut ( (canHigMegBase + 0) << canMegOffset )
#define canIMU ( (canHigMegBase + 1) << canMegOffset )

/*中间级消息(16组)*/
/*0x10 ~ 0x1F(0b01 0000 ~ 0b01 1111)*/
#define canNorMegBase 0x10

#define canSbusIn ((canNorMegBase + 0) << canMegOffset)
#define canGPS ((canNorMegBase + 1) << canMegOffset)
#define canAirspeed ((canNorMegBase + 2) << canMegOffset)

/*最低及消息(16组)*/
/*0x20 ~ 0x2F(0b10 0000 ~ 0b10 1111)*/
#define canLowMegBase 0x20

#define canPower ((canLowMegBase + 0) << canMegOffset)

/*包数据超过8字节，支持拆分，最大支持 8*8 = 64 byte*/
#define canIxdFilter 0x07

#define canMegIxd0 0x00
#define canMegIxd1 0x01
#define canMegIxd2 0x02
#define canMegIxd3 0x03
#define canMegIxd4 0x04
#define canMegIxd5 0x05
#define canMegIxd6 0x06
#define canMegIxd7 0x07

#endif

