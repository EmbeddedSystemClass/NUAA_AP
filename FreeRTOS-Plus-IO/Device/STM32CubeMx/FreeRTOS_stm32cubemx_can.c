/**
 *********************************************************************
 * File Name             :FreeRTOS_stm32cubemx_can.c
 * Description           :This file provides code for Can Peripheral
 *********************************************************************
 * 
 * Copyright (c) 2017 Prince An (www.740129489@qq.com)
 * All rights reserved.
 *
 *
 *********************************************************************
 */

 /* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal_def.h"

/* IO library includes. */
#include "FreeRTOS_IO.h"
#include "IOUtils_Common.h"
#include "FreeRTOS_can.h"

/* Hardware setup peripheral driver includes.  The includes for the UART itself
is already included from FreeRTOS_IO_BSP.h. */

#define canTX_FIFO_LEVEL_MASK		( 0xf00UL )

/* The TEMT bit in the line status register. */
//#define uartTX_BUSY_MASK			( 1UL << 6UL )


/* Stores the transfer control structures that are currently in use by the
supported SPI ports. */
static Transfer_Control_t *pxTxTransferControlStructs[ boardNUM_CANS ] = { NULL };
static Transfer_Control_t *pxRxTransferControlStructs[ boardNUM_CANS ] = { NULL };

static TaskHandle_t xCAN_TX_CMPL_Notify[boardNUM_CANS] = { NULL };
static TaskHandle_t xCAN_RX_CMPL_Notify[boardNUM_UARTS] = { NULL };
static xSemaphoreHandle xCAN_ZC_TX_SIG[boardNUM_CANS] = { NULL };

static uint8_t flagTxIT = 0;

static HAL_StatusTypeDef txDataWrite(CAN_HandleTypeDef *hcan, uint32_t canId, uint8_t len,uint8_t *data);
static HAL_StatusTypeDef txDataSend(CAN_HandleTypeDef *hcan, const size_t len, uint8_t* buff);

portBASE_TYPE FreeRTOS_CAN_open( Peripheral_Control_t * const pxPeripheralControl )
{
CAN_HandleTypeDef * const pxCAN = ( CAN_HandleTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl );
portBASE_TYPE xReturn;
const uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );

	/* Sanity check the peripheral number. */
	if( cPeripheralNumber < boardNUM_CANS )
	{
		pxPeripheralControl->read = FreeRTOS_CAN_read;
		pxPeripheralControl->write = FreeRTOS_CAN_write;
		pxPeripheralControl->ioctl = FreeRTOS_CAN_ioctl;

		/* Setup the pins for the UART being used. */
		//taskENTER_CRITICAL();
		//taskEXIT_CRITICAL();

		xReturn = pdPASS;
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}

static HAL_StatusTypeDef txDataWrite(CAN_HandleTypeDef *hcan, uint32_t canId, uint8_t len,uint8_t *data)
{
	HAL_StatusTypeDef result = HAL_OK;
    hcan->pTxMsg->StdId = canId;   //标准型
	hcan->pTxMsg->IDE   = CAN_ID_STD;  
	hcan->pTxMsg->RTR   = CAN_RTR_DATA; //数据帧
	//SEGGER_RTT_printf(0,"hcan->pTxMsg->StdId = %d\r\n",hcan->pTxMsg->StdId);
	//SEGGER_RTT_printf(0,"hcan->pTxMsg->IDE = %d\r\n",hcan->pTxMsg->IDE);
	//SEGGER_RTT_printf(0,"hcan->pTxMsg->RTR = %d\r\n",hcan->pTxMsg->RTR);
	if( len > 8 )
	{
		hcan->pTxMsg->DLC = 8; 
	}
	else
	{
		hcan->pTxMsg->DLC = len; 
	}

	for(uint8_t i = 0; i < hcan->pTxMsg->DLC; i++)
	{
		hcan->pTxMsg->Data[i] = data[i];
	}
	
    if(flagTxIT == 0)
    {
        result = HAL_CAN_Transmit(hcan,100);
        //SEGGER_RTT_printf(0,"TranResult = %d\r\n",result);
    }
    else
    {
        result = HAL_CAN_Transmit_IT(hcan);
        //SEGGER_RTT_printf(0,"TranResultIT = %d\r\n",result);
    }
   
    if( result != HAL_OK )
    {
      //  SEGGER_RTT_printf(0,"HAL_CAN_Transmit ERROR at %d\r\n", result);
    }	

    return result;
}

static HAL_StatusTypeDef txDataSend(CAN_HandleTypeDef *hcan, const size_t len, uint8_t* buff)
{
    uint8_t lenTemp = len - 4;/*前 4 个用来存储 canID*/
    uint8_t flag = 0;
    uint32_t canID;
    canID = (uint32_t)buff[0] << 24 | buff[1] << 16 | buff[2] << 8 | buff[3]; 
    HAL_StatusTypeDef flagD = HAL_OK;
    while(lenTemp > 0)
    {
        len = (lenTemp > 8) ? 8 : lenTemp;
        canID = canID | flag ;
			//	SEGGER_RTT_printf(0,"message= %d\r\n",message);
			//	SEGGER_RTT_printf(0,"canID= %d\r\n",canID);
        if((flagD = txDataWrite( hcan, canID, len, buff ))!= HAL_OK)
        {
            //SEGGER_RTT_printf(0,"WriteError\r\n");
            return	HAL_ERROR;
        }
        buff += len;
        lenTemp -= len;
        flag ++;
        if(flag > 8)
        {
            return HAL_ERROR;
        }
    }
    //SEGGER_RTT_printf(0,"InterStep");
    return HAL_OK;
}

size_t FreeRTOS_CAN_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
CAN_HandleTypeDef * const pxCAN = ( CAN_HandleTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
int8_t cPeripheralNumber;
    /* 默认为阻塞模式 */
	if( diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
	{
		#if ioconfigUSE_CAN_BLOCKED_TX == 1
		{
			txDataSend( pxCAN, xBytes, (uint8_t*)pvBuffer);
		}
		#endif /* ioconfigUSE_CAN_BLOCKED_TX */
	}
	else
	{

		cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
		pxTxTransferControlStructs[ cPeripheralNumber  ] = diGET_TX_TRANSFER_STRUCT( pxPeripheralControl );

		switch( diGET_TX_TRANSFER_TYPE( pxPeripheralControl ) )
		{
			case ioctlUSE_BLOCKED_TX :

				#if ioconfigUSE_CAN_BLOCKED_TX == 1
				{
					Blocked_TxRx_State_t *pxBlockedTxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
					HAL_StatusTypeDef rslt;
					if (pxBlockedTxState->xMode & ioctlTX_USE_INT_BIT)
					{
						flagTXIT = 1;
                        configASSERT( xCAN_TX_CMPL_Notify[cPeripheralNumber] == NULL );

						/* Store the handle of the calling task. */
						xCAN_TX_CMPL_Notify[cPeripheralNumber] = xTaskGetCurrentTaskHandle();

                        rslt = txDataSend( pxCAN, xBytes, (uint8_t*)pvBuffer);
						
						if (HAL_OK == rslt)
						{
	                         if (!ulTaskNotifyTake( pdTRUE, pxBlockedTxState->xBlockTime))
							{
							    xUART_TX_CMPL_Notify[cPeripheralNumber] = NULL;
							}
						}
					}
					else
					{
						rslt = txDataSend( pxCAN, xBytes, (uint8_t*)pvBuffer);
					}
					if (HAL_OK == rslt)
					{
							xReturn = xBytes;
					}
				}
		    #endif /* ioconfigUSE_UART_BLOCKED_TX */
				break;


			case ioctlUSE_ZERO_COPY_TX :

				#if ioconfigUSE_CAN_ZERO_COPY_TX == 1
				{
				
					Zero_Copy_Tx_State_t *pxZeroCopyTxState = ( Zero_Copy_Tx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
					if (pdPASS == xIOUtilsGetZeroCopyWriteMutex( pxPeripheralControl, ioctlOBTAIN_WRITE_MUTEX, pxZeroCopyTxState->xBlockTime))
					{
						xCAN_ZC_TX_SIG[cPeripheralNumber] = pxZeroCopyTxState->xWriteAccessMutex;
						HAL_StatusTypeDef rslt;
                        rslt = txDataSend( pxCAN, xBytes, (uint8_t*)pvBuffer);
						if (HAL_OK == rslt)
						{
                            xReturn = xBytes;
						}
			         }
				}
				#endif /* ioconfigUSE_UART_ZERO_COPY_TX */
				break;

			case ioctlUSE_CHARACTER_QUEUE_TX :

				#if ioconfigUSE_UART_TX_CHAR_QUEUE == 1
//				{
//					ioutilsBLOCKING_SEND_CHARS_TO_TX_QUEUE
//						(
//							pxPeripheralControl,
//							( pxCAN->State & HAL_CAN_STATE_BUSY_TX ) != HAL_CAN_STATE_BUSY_TX,  /* Peripheral busy condition. */
//							pxUART->THR = ucChar,				/* Peripheral write function. */
//							( ( uint8_t * ) pvBuffer ),			/* Data source. */
//							xBytes, 							/* Number of bytes to be written. */
//							xReturn );

				}
				#endif /* ioconfigUSE_UART_TX_CHAR_QUEUE */
				break;


			default :

				/* Other methods can be implemented here.  For now set the
				stored transfer structure back to NULL as nothing is being
				sent. */
				configASSERT( xReturn );
				pxTxTransferControlStructs[ cPeripheralNumber ] = NULL;

				/* Prevent compiler warnings when the configuration is set such
				that the following parameters are not used. */
				( void ) pvBuffer;
				( void ) xBytes;
				( void ) pxCAN;
				break;
		}
	}

	return xReturn;
}

/*-----------------------------------------------------------*/

size_t FreeRTOS_CAN_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
CAN_HandleTypeDef * const pxCAN = ( CAN_HandleTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
int8_t cPeripheralNumber;

	if( diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
	{
		#if ioconfigUSE_CAN_BLOCKED_RX == 1
		{
			/* No FreeRTOS objects exist to allow reception without blocking
			the task, so just receive by polling.  No semaphore or queue is
			used here, so the application must ensure only one task attempts
			to make a polling read at a time. */
			HAL_CAN_Receive(pxCAN, CAN_FIFO1, 100);
			xReturn = 8;
		}
		#endif /* ioconfigUSE_UART_BLOCKED_RX */
	}
	else
	{
		/* Remember which transfer control structure is being used.
		The Tx interrupt will use this to continue to write data to the
		Tx FIFO/UART until the length member of the structure reaches
		zero. */
		cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
		pxRxTransferControlStructs[ cPeripheralNumber  ] = diGET_RX_TRANSFER_STRUCT( pxPeripheralControl );

		switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
		{
			case ioctlUSE_BLOCKED_RX :

				#if ioconfigUSE_CAN_BLOCKED_RX == 1
				{
					Blocked_TxRx_State_t *pxBlockedRxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
					HAL_StatusTypeDef rslt;
					if (pxBlockedRxState->xMode & ioctlRX_USE_INT_BIT)
					{
						configASSERT( xCAN_RX_CMPL_Notify[cPeripheralNumber] == NULL );

						/* Store the handle of the calling task. */
						xCAN_RX_CMPL_Notify[cPeripheralNumber] = xTaskGetCurrentTaskHandle();

						rslt = HAL_CAN_Receive_IT( pxCAN, CAN_FIFO1);
						
						if (HAL_OK == rslt)
						{
							if (ulTaskNotifyTake( pdTRUE, pxBlockedRxState->xBlockTime))
							{
								xReturn = 8;
							}
							else
							{
							  xCAN_RX_CMPL_Notify[cPeripheralNumber] = NULL;
							}
						}
					}
					else
					{
						rslt = HAL_CAN_Receive(pxCAN, CAN_FIFO1, 100);
						if (HAL_OK == rslt)
						{
							xReturn = 8;
						}
					}
				}

		    #endif /* ioconfigUSE_CAN_BLOCKED_RX */
				break;

			case ioctlUSE_CIRCULAR_BUFFER_RX :

				#if ioconfigUSE_CAN_CIRCULAR_BUFFER_RX == 1
				{

				}
				#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
				break;


			case ioctlUSE_CHARACTER_QUEUE_RX :

				#if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
				{
					//xReturn = xIOUtilsReceiveCharsFromRxQueue( pxPeripheralControl, ( uint8_t * ) pvBuffer, xBytes );
				}
				#endif /* ioconfigUSE_UART_RX_CHAR_QUEUE */
				break;


			default :

				/* Other methods can be implemented here. */
				configASSERT( xReturn );
				pxTxTransferControlStructs[ cPeripheralNumber ] = NULL;

				/* Prevent compiler warnings when the configuration is set such
				that the following parameters are not used. */
				( void ) pvBuffer;
				( void ) xBytes;
				( void ) pxUART;
				break;
		}
	}

	return xReturn;
}

portBASE_TYPE FreeRTOS_CAN_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
uint32_t ulValue = ( uint32_t ) pvValue;
const int8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
CAN_HandleTypeDef * pxCAN = ( CAN_HandleTypeDef * ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
portBASE_TYPE xReturn = pdPASS;

	taskENTER_CRITICAL();
	{
		switch( ulRequest )
		{
			case ioctlUSE_MODE :
				if (diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ))
				{
					switch( diGET_TX_TRANSFER_TYPE( pxPeripheralControl ) )
					{
						case ioctlUSE_BLOCKED_TX :

							#if ioconfigUSE_UART_BLOCKED_TX == 1
							{
								Blocked_TxRx_State_t *pxPlledTxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
								pxPlledTxState->xMode = ulValue;
							}
							#endif /* ioconfigUSE_CAN_BLOCKED_TX */
							break;
						case ioctlUSE_ZERO_COPY_TX :

							#if ioconfigUSE_ZERO_COPY_TX == 1
							{
								Zero_Copy_Tx_State_t *pxZeroCopyTxState = ( Zero_Copy_Tx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
								pxZeroCopyTxState->xMode = ulValue;
							}
							#endif /* ioconfigUSE_CAN_BLOCKED_TX */
							break;
					}	
			  }
				if (diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ))
				{
					switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
					{
						case ioctlUSE_BLOCKED_RX :

							#if ioconfigUSE_CAN_BLOCKED_RX == 1
							{
								Blocked_TxRx_State_t *pxPlledRxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
								pxPlledRxState->xMode = ulValue;
							}
							#endif /* ioconfigUSE_UART_BLOCKED_RX */
							break;
					}	
				}
				break;
				
				case ioctlCLEAR_RX_BUFFER:
				if (diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ))
				{
					switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
					{
						case ioctlUSE_BLOCKED_RX :

							#if ioconfigUSE_CAN_BLOCKED_RX == 1
							{
								
							}
							#endif /* ioconfigUSE_CAN_BLOCKED_RX */
							break;
													
						case ioctlUSE_CIRCULAR_BUFFER_RX :
							#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
							{
								
							}
							#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
							break;
					}	
				}
				else
				{
				  
				}
				break;


			default :

				xReturn = pdFAIL;
				break;
		}
	}
	taskEXIT_CRITICAL();

	return xReturn;
}
