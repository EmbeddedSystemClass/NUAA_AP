/**
  ******************************************************************************
  * File Name          : IOUtils_BlockedTxRx.h
  * Description        : This file provides code for ioctlUSE_BLOCKED_TXRX
  ******************************************************************************
  *
  * Copyright (c) 2017 Zheng GONG(matt@matthewgong.com) 
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

#ifndef IOUTILS_BLOCKED_TXRX_H
#define IOUTILS_BLOCKED_TXRX_H

/* The transfer structure used when a zero copy method is used for
transmission. */
typedef struct xBLOCKED_TXRX_STATE
{
  uint32_t xMode;
  portTickType xBlockTime;			/* The amount of time a task should be held in the Blocked state (not using CPU time) to wait for data to become available when it attempts a read. */
} Blocked_TxRx_State_t;


/* Prototypes of functions that are for internal use only. */
portBASE_TYPE xIOUtilsConfigureBlockedTx( Peripheral_Control_t * const pxPeripheralControl );
portBASE_TYPE xIOUtilsConfigureBlockedRx( Peripheral_Control_t * const pxPeripheralControl );
void xIOUtilsSetBlockedTxRxTimeout( Peripheral_Control_t * const pxPeripheralControl, const portTickType xMaxWaitTime );

#endif /* IOUTILS_BLOCKED_TXRX_H */




