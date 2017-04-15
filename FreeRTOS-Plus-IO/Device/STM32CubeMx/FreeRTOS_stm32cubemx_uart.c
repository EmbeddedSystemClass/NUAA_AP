/*
 * FreeRTOS+IO V1.0.1 (C) 2012 Real Time Engineers ltd.
 *
 * FreeRTOS+IO is an add-on component to FreeRTOS.  It is not, in itself, part
 * of the FreeRTOS kernel.  FreeRTOS+IO is licensed separately from FreeRTOS,
 * and uses a different license to FreeRTOS.  FreeRTOS+IO uses a dual license
 * model, information on which is provided below:
 *
 * - Open source licensing -
 * FreeRTOS+IO is a free download and may be used, modified and distributed
 * without charge provided the user adheres to version two of the GNU General
 * Public license (GPL) and does not remove the copyright notice or this text.
 * The GPL V2 text is available on the gnu.org web site, and on the following
 * URL: http://www.FreeRTOS.org/gpl-2.0.txt
 *
 * - Commercial licensing -
 * Businesses and individuals who wish to incorporate FreeRTOS+IO into
 * proprietary software for redistribution in any form must first obtain a low
 * cost commercial license - and in-so-doing support the maintenance, support
 * and further development of the FreeRTOS+IO product.  Commercial licenses can
 * be obtained from http://shop.freertos.org and do not require any source files
 * to be changed.
 *
 * FreeRTOS+IO is distributed in the hope that it will be useful.  You cannot
 * use FreeRTOS+IO unless you agree that you use the software 'as is'.
 * FreeRTOS+IO is provided WITHOUT ANY WARRANTY; without even the implied
 * warranties of NON-INFRINGEMENT, MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE. Real Time Engineers Ltd. disclaims all conditions and terms, be they
 * implied, expressed, or statutory.
 *
 * 1 tab == 4 spaces!
 *
 * http://www.FreeRTOS.org
 * http://www.FreeRTOS.org/FreeRTOS-Plus
 *
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
#include "FreeRTOS_uart.h"

/* Hardware setup peripheral driver includes.  The includes for the UART itself
is already included from FreeRTOS_IO_BSP.h. */

/* The bits in the FIFOLVL register that represent the Tx Fifo level. */
#define uartTX_FIFO_LEVEL_MASK		( 0xf00UL )

/* The TEMT bit in the line status register. */
#define uartTX_BUSY_MASK			( 1UL << 6UL )

/*-----------------------------------------------------------*/

/* Stores the transfer control structures that are currently in use by the
supported UART ports. */
static Transfer_Control_t *pxTxTransferControlStructs[ boardNUM_UARTS ] = { NULL };
static Transfer_Control_t *pxRxTransferControlStructs[ boardNUM_UARTS ] = { NULL };

static TaskHandle_t xUART_TX_CMPL_Notify[boardNUM_UARTS] = { NULL };
static TaskHandle_t xUART_RX_CMPL_Notify[boardNUM_UARTS] = { NULL };
static xSemaphoreHandle xUART_ZC_TX_SIG[boardNUM_UARTS] = { NULL };


/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_UART_open( Peripheral_Control_t * const pxPeripheralControl )
{
UART_HandleTypeDef * const pxUART = ( UART_HandleTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl );
portBASE_TYPE xReturn;
const uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );

	/* Sanity check the peripheral number. */
	if( cPeripheralNumber < boardNUM_UARTS )
	{
		pxPeripheralControl->read = FreeRTOS_UART_read;
		pxPeripheralControl->write = FreeRTOS_UART_write;
		pxPeripheralControl->ioctl = FreeRTOS_UART_ioctl;

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
/*-----------------------------------------------------------*/

size_t FreeRTOS_UART_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
UART_HandleTypeDef * const pxUART = ( UART_HandleTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
int8_t cPeripheralNumber;

	if( diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
	{
		#if ioconfigUSE_UART_BLOCKED_TX == 1
		{
			/* No FreeRTOS objects exist to allow transmission without blocking
			the	task, so just send out by polling.  No semaphore or queue is
			used here, so the application must ensure only one task attempts to
			make a polling write at a time. */
			HAL_UART_Transmit(pxUART, (uint8_t *)pvBuffer, xBytes, osWaitForever);
		}
		#endif /* ioconfigUSE_UART_BLOCKED_TX */
	}
	else
	{
		/* Remember which transfer control structure is being used.
		The Tx interrupt will use this to continue to write data to the
		Tx FIFO/UART until the length member of the structure reaches
		zero. */
		cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
		pxTxTransferControlStructs[ cPeripheralNumber  ] = diGET_TX_TRANSFER_STRUCT( pxPeripheralControl );

		switch( diGET_TX_TRANSFER_TYPE( pxPeripheralControl ) )
		{
			case ioctlUSE_BLOCKED_TX :

				#if ioconfigUSE_UART_BLOCKED_TX == 1
				{
					Blocked_TxRx_State_t *pxBlockedTxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
					HAL_StatusTypeDef rslt;
					if (pxBlockedTxState->xMode & (ioctlTX_USE_DMA_BIT | ioctlTX_USE_INT_BIT))
					{
						configASSERT( xUART_TX_CMPL_Notify[cPeripheralNumber] == NULL );

						/* Store the handle of the calling task. */
						xUART_TX_CMPL_Notify[cPeripheralNumber] = xTaskGetCurrentTaskHandle();

						if (pxBlockedTxState->xMode & ioctlTX_USE_DMA_BIT)
						{
							rslt = HAL_UART_Transmit_DMA(pxUART, (uint8_t *)pvBuffer, xBytes);
						}
						else
						{
							rslt = HAL_UART_Transmit_IT(pxUART, (uint8_t *)pvBuffer, xBytes);
						}
						if (HAL_OK == rslt)
						{
	            if (!ulTaskNotifyTake( pdTRUE, pxBlockedTxState->xBlockTime))
							{
								HAL_UART_AbortTransmit_IT(pxUART);
							  xUART_TX_CMPL_Notify[cPeripheralNumber] = NULL;
							}
						}
					}
					else
					{
						rslt = HAL_UART_Transmit(pxUART, (uint8_t *)pvBuffer, xBytes, osWaitForever);
					}
					if (HAL_OK == rslt)
					{
							xReturn = xBytes;
					}
				}
		    #endif /* ioconfigUSE_UART_BLOCKED_TX */
				break;


			case ioctlUSE_ZERO_COPY_TX :

				#if ioconfigUSE_UART_ZERO_COPY_TX == 1
				{
					/* The implementation of the zero copy write uses a semaphore
					to indicate whether a write is complete (and so the buffer
					being written free again) or not.  The semantics of using a
					zero copy write dictate that a zero copy write can only be
					attempted by a task, once the semaphore has been successfully
					obtained by that task.  This ensure that only one task can
					perform a zero copy write at any one time.  Ensure the semaphore
					is not currently available, if this function has been called
					without it being obtained first then it is an error. */
					Zero_Copy_Tx_State_t *pxZeroCopyTxState = ( Zero_Copy_Tx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
					if (pdPASS == xIOUtilsGetZeroCopyWriteMutex( pxPeripheralControl, ioctlOBTAIN_WRITE_MUTEX, pxZeroCopyTxState->xBlockTime))
					{
						xUART_ZC_TX_SIG[cPeripheralNumber] = pxZeroCopyTxState->xWriteAccessMutex;
						HAL_StatusTypeDef rslt;
						if (pxZeroCopyTxState->xMode & ioctlTX_USE_DMA_BIT)
						{
							rslt = HAL_UART_Transmit_DMA(pxUART, (uint8_t *)pvBuffer, xBytes);
						}
						else
						{
							rslt = HAL_UART_Transmit_IT(pxUART, (uint8_t *)pvBuffer, xBytes);
						}
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
				{
					/* The queue allows multiple tasks to attempt to write
					bytes, but ensures only the highest priority of these tasks
					will actually succeed.  If two tasks of equal priority
					attempt to write simultaneously, then the application must
					ensure mutual exclusion, as time slicing could result in
					the strings being sent to the queue being interleaved. */
					ioutilsBLOCKING_SEND_CHARS_TO_TX_QUEUE
						(
							pxPeripheralControl,
							( pxUART->LSR & uartTX_BUSY_MASK ) == uartTX_BUSY_MASK,  /* Peripheral busy condition. */
							pxUART->THR = ucChar,				/* Peripheral write function. */
							( ( uint8_t * ) pvBuffer ),			/* Data source. */
							xBytes, 							/* Number of bytes to be written. */
							xReturn );
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
				( void ) pxUART;
				break;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

size_t FreeRTOS_UART_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
UART_HandleTypeDef * const pxUART = ( UART_HandleTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
int8_t cPeripheralNumber;

	if( diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
	{
		#if ioconfigUSE_UART_BLOCKED_RX == 1
		{
			/* No FreeRTOS objects exist to allow reception without blocking
			the task, so just receive by polling.  No semaphore or queue is
			used here, so the application must ensure only one task attempts
			to make a polling read at a time. */
			HAL_UART_Receive(pxUART, (uint8_t *)pvBuffer, xBytes, osWaitForever);
			xReturn = xBytes;
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

				#if ioconfigUSE_UART_BLOCKED_RX == 1
				{
					Blocked_TxRx_State_t *pxBlockedRxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
					HAL_StatusTypeDef rslt;
					if (pxBlockedRxState->xMode & (ioctlRX_USE_DMA_BIT | ioctlRX_USE_INT_BIT))
					{
						configASSERT( xUART_TX_CMPL_Notify[cPeripheralNumber] == NULL );

						/* Store the handle of the calling task. */
						xUART_RX_CMPL_Notify[cPeripheralNumber] = xTaskGetCurrentTaskHandle();

						if (pxBlockedRxState->xMode & ioctlRX_USE_DMA_BIT)
						{
							rslt = HAL_UART_Receive_DMA(pxUART, (uint8_t *)pvBuffer, xBytes);
						}
						else
						{
							rslt = HAL_UART_Receive_IT(pxUART, (uint8_t *)pvBuffer, xBytes);
						}
						if (HAL_OK == rslt)
						{
							if (ulTaskNotifyTake( pdTRUE, pxBlockedRxState->xBlockTime))
							{
								xReturn = xBytes;
							}
							else
							{
								HAL_UART_AbortReceive_IT(pxUART);
							  xUART_RX_CMPL_Notify[cPeripheralNumber] = NULL;
							}
						}
					}
					else
					{
						rslt = HAL_UART_Receive(pxUART, (uint8_t *)pvBuffer, xBytes, osWaitForever);
						if (HAL_OK == rslt)
						{
							xReturn = xBytes;
						}
						else if (HAL_TIMEOUT == rslt)
						{
							xReturn = pxUART->RxXferSize - pxUART->RxXferCount;
						}
					}
				}
		    #endif /* ioconfigUSE_UART_BLOCKED_RX */
				break;


			case ioctlUSE_CIRCULAR_BUFFER_RX :

				#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
				{
					Circular_Buffer_Rx_State_t *pxRxState = ( Circular_Buffer_Rx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
					TickType_t ticks = pxRxState->xBlockTime;
					uint16_t uart_rx_head_idx;
					uint8_t *target = pvBuffer;
					for (size_t i = 0u; i < xBytes; ++i)
					{
            uart_rx_head_idx = pxRxState->usBufferLength-__HAL_DMA_GET_COUNTER(pxUART->hdmarx);
						while ((uart_rx_head_idx == pxRxState->usNextReadIndex) && (ticks > 0u))
						{
							vTaskDelay(1);
							--ticks;
							uart_rx_head_idx = pxRxState->usBufferLength-__HAL_DMA_GET_COUNTER(pxUART->hdmarx);
						}
						if (uart_rx_head_idx == pxRxState->usNextReadIndex)
						{
						  break;
						}
						target[xReturn++] = pxRxState->pucBufferStart[pxRxState->usNextReadIndex++];
						if (pxRxState->usNextReadIndex >= pxRxState->usBufferLength)
						{
							pxRxState->usNextReadIndex = 0u;
						}
					}
				}
				#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
				break;


			case ioctlUSE_CHARACTER_QUEUE_RX :

				#if ioconfigUSE_UART_RX_CHAR_QUEUE == 1
				{
					/* The queue allows multiple tasks to attempt to read
					bytes, but ensures only the highest priority of these
					tasks will actually receive bytes.  If two tasks of equal
					priority attempt to read simultaneously, then the
					application must ensure mutual exclusion, as time slicing
					could result in the string being received being partially
					received by each task. */
					xReturn = xIOUtilsReceiveCharsFromRxQueue( pxPeripheralControl, ( uint8_t * ) pvBuffer, xBytes );
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

/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_UART_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
uint32_t ulValue = ( uint32_t ) pvValue;
const int8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
UART_HandleTypeDef * pxUART = ( UART_HandleTypeDef * ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
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
							#endif /* ioconfigUSE_UART_BLOCKED_TX */
							break;
						case ioctlUSE_ZERO_COPY_TX :

							#if ioconfigUSE_ZERO_COPY_TX == 1
							{
								Zero_Copy_Tx_State_t *pxZeroCopyTxState = ( Zero_Copy_Tx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
								pxZeroCopyTxState->xMode = ulValue;
							}
							#endif /* ioconfigUSE_UART_BLOCKED_TX */
							break;
					}	
			  }
				if (diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ))
				{
					switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
					{
						case ioctlUSE_BLOCKED_RX :

							#if ioconfigUSE_UART_BLOCKED_RX == 1
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

							#if ioconfigUSE_UART_BLOCKED_RX == 1
							{
									__HAL_UART_CLEAR_FLAG(pxUART, UART_FLAG_RXNE);
							}
							#endif /* ioconfigUSE_UART_BLOCKED_RX */
							break;
													
						case ioctlUSE_CIRCULAR_BUFFER_RX :
							#if ioconfigUSE_UART_CIRCULAR_BUFFER_RX == 1
							{
								Circular_Buffer_Rx_State_t *pxRxState = ( Circular_Buffer_Rx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
                pxRxState->usNextReadIndex = pxRxState->usBufferLength-__HAL_DMA_GET_COUNTER(pxUART->hdmarx);
							}
							#endif /* ioconfigUSE_UART_CIRCULAR_BUFFER_RX */
							break;
					}	
				}
				else
				{
				  __HAL_UART_CLEAR_FLAG(pxUART, UART_FLAG_RXNE);
				}
				break;

				case ioctlSTART_CIRCULAR_BUFFER_RX_DMA:
				{
						Circular_Buffer_Rx_State_t *pxRxState = ( Circular_Buffer_Rx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
				    HAL_UART_Receive_DMA(pxUART, pxRxState->pucBufferStart, pxRxState->usBufferLength);
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

/*-----------------------------------------------------------*/

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  TaskHandle_t cur_task;
	xSemaphoreHandle cur_sig;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t idx;

	idx = boardNUM_UARTS;

	if (huart == &huart2)
	{
		idx = 2;
	}
	else if (huart == &huart3)
	{
		idx = 3;
	}
	else if (huart == &huart5)
	{
		idx = 5;
	}
	else if (huart == &huart1)
	{
		idx = 1;
	}
	if (idx < boardNUM_UARTS)
	{
	  cur_task = xUART_TX_CMPL_Notify[idx];
		cur_sig = xUART_ZC_TX_SIG[idx];
	}
	else
	{
		cur_task = NULL;
		cur_sig = NULL;
	}

	if( cur_task != NULL )
	{
		/* Notify the task that the transmission is complete. */
		vTaskNotifyGiveFromISR( cur_task, &xHigherPriorityTaskWoken );
	  xUART_TX_CMPL_Notify[idx] = NULL;
	}
	else if (cur_sig != NULL)
	{
	  xSemaphoreGiveFromISR( cur_sig, &xHigherPriorityTaskWoken );
	}

	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
	should be performed to ensure the interrupt returns directly to the highest
	priority task.  The macro used for this purpose is dependent on the port in
	use and may be called portEND_SWITCHING_ISR(). */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  TaskHandle_t cur_task;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t idx;

	cur_task = NULL;

	idx = boardNUM_UARTS;

	if (huart == &huart2)
	{
		idx = 2;
	}
	else if (huart == &huart3)
	{
		idx = 3;
	}
	else if (huart == &huart5)
	{
		idx = 5;
	}
	else if (huart == &huart1)
	{
		idx = 1;
	}
	if (idx < boardNUM_UARTS)
	{
	  cur_task = xUART_RX_CMPL_Notify[idx];
	}
	else
	{
		cur_task = NULL;
	}

	if ( cur_task != NULL )
	{
		/* Notify the task that the transmission is complete. */
		vTaskNotifyGiveFromISR( cur_task, &xHigherPriorityTaskWoken );
		xUART_RX_CMPL_Notify[idx] = NULL;
	}

	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
	should be performed to ensure the interrupt returns directly to the highest
	priority task.  The macro used for this purpose is dependent on the port in
	use and may be called portEND_SWITCHING_ISR(). */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//  TaskHandle_t cur_task_rx;
//  TaskHandle_t cur_task_tx;
//	xSemaphoreHandle cur_sig;
//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//	BaseType_t idx;
//	

//	idx = boardNUM_UARTS;

//	if (huart == &huart2)
//	{
//		idx = 2;
//	}
//	else if (huart == &huart3)
//	{
//		idx = 3;
//	}
//	else if (huart == &huart5)
//	{
//		idx = 5;
//	}
//	else if (huart == &huart1)
//	{
//		idx = 1;
//	}
//	if (idx < boardNUM_UARTS)
//	{
//	  cur_task_tx = xUART_TX_CMPL_Notify[idx];
//	  cur_task_rx = xUART_RX_CMPL_Notify[idx];
//		cur_sig = xUART_ZC_TX_SIG[idx];
//	}
//	else
//	{
//		cur_task_tx = NULL;
//		cur_task_rx = NULL;
//		cur_sig = NULL;
//	}

//	if( cur_task_tx != NULL )
//	{
//		/* Notify the task that the transmission is complete. */
//		vTaskNotifyGiveFromISR( cur_task_tx, &xHigherPriorityTaskWoken );
//	  xUART_TX_CMPL_Notify[idx] = NULL;
//	}
//	else if( cur_task_rx != NULL )
//	{
//		/* Notify the task that the transmission is complete. */
//		vTaskNotifyGiveFromISR( cur_task_rx, &xHigherPriorityTaskWoken );
//	  xUART_RX_CMPL_Notify[idx] = NULL;
//	}
//	else if (cur_sig != NULL)
//	{
//	  xSemaphoreGiveFromISR( cur_sig, &xHigherPriorityTaskWoken );
//	}

//	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
//	should be performed to ensure the interrupt returns directly to the highest
//	priority task.  The macro used for this purpose is dependent on the port in
//	use and may be called portEND_SWITCHING_ISR(). */
//	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

//}

int fputc(int ch, FILE *f)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  //UDT_Buffer_Push(&cmd_tx, ch);
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

