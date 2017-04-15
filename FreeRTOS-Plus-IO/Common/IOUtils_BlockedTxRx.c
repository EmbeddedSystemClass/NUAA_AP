/**
  ******************************************************************************
  * File Name          : IOUtils_BlockedTxRx.c
  * Description        : This file provides code for ioctlUSE_BLOCKED_TXRX
  ******************************************************************************
  *
  * Copyright (c) 2017 Zheng GONG(matt@matthewgong.com) 
  * All rights reserved.
  *
  *
  ******************************************************************************
  */

/* Standard includes. */
#include "string.h"

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Device specific library includes. */
#include "FreeRTOS_DriverInterface.h"
#include "IOUtils_Common.h"

/*-----------------------------------------------------------*/

portBASE_TYPE xIOUtilsConfigureBlockedTx( Peripheral_Control_t * const pxPeripheralControl )
{
portBASE_TYPE xReturn = pdFAIL;
Blocked_TxRx_State_t *pxBlockedState;

	/* A peripheral is going to use a Blocked_TxRx_State_t structure to control
	transmission. */
	vIOUtilsCreateTransferControlStructure( &( pxPeripheralControl->pxTxControl ) );
	configASSERT( pxPeripheralControl->pxTxControl );

	if( pxPeripheralControl->pxTxControl != NULL )
	{
		/* Create the necessary structure. */
		pxBlockedState = pvPortMalloc( sizeof( Blocked_TxRx_State_t ) );

		if( pxBlockedState != NULL )
		{
			pxBlockedState->xMode = 0u;
			pxBlockedState->xBlockTime = 100;

			pxPeripheralControl->pxTxControl->pvTransferState = ( void * ) pxBlockedState;
			pxPeripheralControl->pxTxControl->ucType = ioctlUSE_BLOCKED_TX;
			xReturn = pdPASS;
		}
    else
		{
			/* The TxRx structure, or a member it contains,  could not be created,
			so the TxRx control structure (which should point to it) should also
			be deleted. */
			vPortFree( pxPeripheralControl->pxTxControl );
			pxPeripheralControl->pxTxControl = NULL;
		}
	}

	return xReturn;
}

portBASE_TYPE xIOUtilsConfigureBlockedRx( Peripheral_Control_t * const pxPeripheralControl )
{
portBASE_TYPE xReturn = pdFAIL;
Blocked_TxRx_State_t *pxBlockedState;

	/* A peripheral is going to use a Blocked_TxRx_State_t structure to control
	transmission. */
	vIOUtilsCreateTransferControlStructure( &( pxPeripheralControl->pxRxControl ) );
	configASSERT( pxPeripheralControl->pxRxControl );

	if( pxPeripheralControl->pxRxControl != NULL )
	{
		/* Create the necessary structure. */
		pxBlockedState = pvPortMalloc( sizeof( Blocked_TxRx_State_t ) );

		if( pxBlockedState != NULL )
		{
			pxBlockedState->xMode = 0u;
			pxBlockedState->xBlockTime = portMAX_DELAY;

			pxPeripheralControl->pxRxControl->pvTransferState = ( void * ) pxBlockedState;
			pxPeripheralControl->pxRxControl->ucType = ioctlUSE_BLOCKED_RX;
			xReturn = pdPASS;
		}
    else
		{
			/* The TxRx structure, or a member it contains,  could not be created,
			so the TxRx control structure (which should point to it) should also
			be deleted. */
			vPortFree( pxPeripheralControl->pxRxControl );
			pxPeripheralControl->pxRxControl = NULL;
		}
	}

	return xReturn;
}

/*-----------------------------------------------------------*/

void xIOUtilsSetBlockedTxRxTimeout( Peripheral_Control_t * const pxPeripheralControl, const portTickType xMaxWaitTime )
{
Transfer_Control_t *pxTransferControlState = pxPeripheralControl->pxRxControl;
Blocked_TxRx_State_t *pxBlockedTxRxState;

	pxBlockedTxRxState = ( Blocked_TxRx_State_t * ) ( pxTransferControlState->pvTransferState );
	pxBlockedTxRxState->xBlockTime = xMaxWaitTime;
}

/*-----------------------------------------------------------*/












