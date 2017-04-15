/**
 *********************************************************************
 * File Name             :FreeRTOS_can.h
 * Description           :This file provides code for Can Peripheral
 *********************************************************************
 * 
 * Copyright (c) 2017 Prince An (www.740129489@qq.com)
 * All rights reserved.
 *
 *
 *********************************************************************
 */

#ifndef __FREERTOS_IO_CAN_H
#define __FREERTOS_IO_CAN_H

#include "FreeRTOS_DriverInterface.h"
#include "portmacro.h"/*提供portBASE_TYPE类型*/

/* These are not public functions.  Do not call these functions directly.  Call
FreeRTOS_Open(), FreeRTOS_write(), FreeRTOS_read() and FreeRTOS_ioctl() only. */

/* 上层函数调用 FreeRTOS_Open(), FreeRTOS_write(), FreeRTOS_Read(), FreeRTOS_ioctl() 函数*/

portBASE_TYPE FreeRTOS_CAN_open( Peripheral_Control_t * const pxPeripheralControl );
size_t FreeRTOS_CAN_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes );
size_t FreeRTOS_CAN_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes );
portBASE_TYPE FreeRTOS_CAN_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue );

#endif




