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
#include "FreeRTOS_spi.h"

/* Hardware setup peripheral driver includes.  The includes for the SPI itself
is already included from FreeRTOS_IO_BSP.h. */

/* The bits in the FIFOLVL register that represent the Tx Fifo level. */
#define uartTX_FIFO_LEVEL_MASK		( 0xf00UL )

/*-----------------------------------------------------------*/

/* Stores the transfer control structures that are currently in use by the
supported SPI ports. */
static Transfer_Control_t *pxTxTransferControlStructs[ boardNUM_SPIS ] = { NULL };
static Transfer_Control_t *pxRxTransferControlStructs[ boardNUM_SPIS ] = { NULL };

static TaskHandle_t xSPI_TXRX_CMPL_Notify[boardNUM_SPIS] = { NULL };
static xSemaphoreHandle xSPI_ZC_TX_SIG[boardNUM_SPIS] = { NULL };


/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_SPI_open( Peripheral_Control_t * const pxPeripheralControl )
{
SPI_DeviceTypeDef * const pxSPI = ( SPI_DeviceTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( pxPeripheralControl );
portBASE_TYPE xReturn;
const uint8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );

	/* Sanity check the peripheral number. */
	if( cPeripheralNumber < boardNUM_SPIS )
	{
		pxPeripheralControl->read = FreeRTOS_SPI_read;
		pxPeripheralControl->write = FreeRTOS_SPI_write;
		pxPeripheralControl->ioctl = FreeRTOS_SPI_ioctl;

		/* Setup the pins for the SPI being used. */
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

size_t FreeRTOS_SPI_write( Peripheral_Descriptor_t const pxPeripheral, const void *pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
SPI_DeviceTypeDef * const pxSPI = ( SPI_DeviceTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
int8_t cPeripheralNumber;

	if( diGET_TX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
	{
		#if ioconfigUSE_SPI_BLOCKED_TX == 1
		{
			/* No FreeRTOS objects exist to allow transmission without blocking
			the	task, so just send out by polling.  No semaphore or queue is
			used here, so the application must ensure only one task attempts to
			make a polling write at a time. */
			HAL_GPIO_WritePin(pxSPI->GPIOx_CS, pxSPI->GPIO_Pin_CS, GPIO_PIN_RESET);
			HAL_SPI_TransmitReceive(pxSPI->hspi, (uint8_t *)pvBuffer, (uint8_t *)pvBuffer, xBytes, osWaitForever);
			HAL_GPIO_WritePin(pxSPI->GPIOx_CS, pxSPI->GPIO_Pin_CS, GPIO_PIN_SET);
		}
		#endif /* ioconfigUSE_SPI_BLOCKED_TX */
	}
	else
	{
		/* Remember which transfer control structure is being used.
		The Tx interrupt will use this to continue to write data to the
		Tx FIFO/SPI until the length member of the structure reaches
		zero. */
		cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
		pxTxTransferControlStructs[ cPeripheralNumber  ] = diGET_TX_TRANSFER_STRUCT( pxPeripheralControl );

		switch( diGET_TX_TRANSFER_TYPE( pxPeripheralControl ) )
		{
			case ioctlUSE_BLOCKED_TX :

				#if ioconfigUSE_SPI_BLOCKED_TX == 1
				{
					Blocked_TxRx_State_t *pxBlockedTxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
					HAL_StatusTypeDef rslt;
					if (pxBlockedTxState->xMode & (ioctlTX_USE_DMA_BIT | ioctlTX_USE_INT_BIT))
					{
						configASSERT( xSPI_TXRX_CMPL_Notify[cPeripheralNumber] == NULL );

						/* Store the handle of the calling task. */
						xSPI_TXRX_CMPL_Notify[cPeripheralNumber] = xTaskGetCurrentTaskHandle();

			      HAL_GPIO_WritePin(pxSPI->GPIOx_CS, pxSPI->GPIO_Pin_CS, GPIO_PIN_RESET);
						if (pxBlockedTxState->xMode & ioctlTX_USE_DMA_BIT)
						{
							rslt = HAL_SPI_TransmitReceive_DMA(pxSPI->hspi, (uint8_t *)pvBuffer, (uint8_t *)pvBuffer, xBytes);
						}
						else
						{
							rslt = HAL_SPI_TransmitReceive_IT(pxSPI->hspi, (uint8_t *)pvBuffer, (uint8_t *)pvBuffer, xBytes);
						}
						if (HAL_OK == rslt)
						{
	            if (!ulTaskNotifyTake( pdTRUE, pxBlockedTxState->xBlockTime))
							{
							  xSPI_TXRX_CMPL_Notify[cPeripheralNumber] = NULL;
							}
						}
			      HAL_GPIO_WritePin(pxSPI->GPIOx_CS, pxSPI->GPIO_Pin_CS, GPIO_PIN_SET);
					}
					else
					{
			      HAL_GPIO_WritePin(pxSPI->GPIOx_CS, pxSPI->GPIO_Pin_CS, GPIO_PIN_RESET);
						rslt = HAL_SPI_TransmitReceive(pxSPI->hspi, (uint8_t *)pvBuffer, (uint8_t *)pvBuffer, xBytes, osWaitForever);
			      HAL_GPIO_WritePin(pxSPI->GPIOx_CS, pxSPI->GPIO_Pin_CS, GPIO_PIN_SET);
					}
					if (HAL_OK == rslt)
					{
							xReturn = xBytes;
					}
				}
		    #endif /* ioconfigUSE_SPI_BLOCKED_TX */
				break;


			case ioctlUSE_ZERO_COPY_TX :

				#if ioconfigUSE_SPI_ZERO_COPY_TX == 1
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
						xSPI_ZC_TX_SIG[cPeripheralNumber] = pxZeroCopyTxState->xWriteAccessMutex;
						HAL_StatusTypeDef rslt;
						if (pxZeroCopyTxState->xMode & ioctlTX_USE_DMA_BIT)
						{
							rslt = HAL_SPI_Transmit_DMA(pxSPI, (uint8_t *)pvBuffer, xBytes);
						}
						else
						{
							rslt = HAL_SPI_Transmit_IT(pxSPI, (uint8_t *)pvBuffer, xBytes);
						}
						if (HAL_OK == rslt)
						{
								xReturn = xBytes;
						}
			    }
				}
				#endif /* ioconfigUSE_SPI_ZERO_COPY_TX */
				break;


			case ioctlUSE_CHARACTER_QUEUE_TX :

				#if ioconfigUSE_SPI_TX_CHAR_QUEUE == 1
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
							( pxSPI->LSR & uartTX_BUSY_MASK ) == uartTX_BUSY_MASK,  /* Peripheral busy condition. */
							pxSPI->THR = ucChar,				/* Peripheral write function. */
							( ( uint8_t * ) pvBuffer ),			/* Data source. */
							xBytes, 							/* Number of bytes to be written. */
							xReturn );
				}
				#endif /* ioconfigUSE_SPI_TX_CHAR_QUEUE */
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
				( void ) pxSPI;
				break;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

size_t FreeRTOS_SPI_read( Peripheral_Descriptor_t const pxPeripheral, void * const pvBuffer, const size_t xBytes )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
size_t xReturn = 0U;
SPI_DeviceTypeDef * const pxSPI = ( SPI_DeviceTypeDef * const ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
int8_t cPeripheralNumber;

	if( diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ) == NULL )
	{
		#if ioconfigUSE_SPI_BLOCKED_RX == 1
		{
			/* No FreeRTOS objects exist to allow reception without blocking
			the task, so just receive by polling.  No semaphore or queue is
			used here, so the application must ensure only one task attempts
			to make a polling read at a time. */
			HAL_SPI_Receive(pxSPI, (uint8_t *)pvBuffer, xBytes, osWaitForever);
			xReturn = xBytes;
		}
		#endif /* ioconfigUSE_SPI_BLOCKED_RX */
	}
	else
	{
		/* Remember which transfer control structure is being used.
		The Tx interrupt will use this to continue to write data to the
		Tx FIFO/SPI until the length member of the structure reaches
		zero. */
		cPeripheralNumber = diGET_PERIPHERAL_NUMBER( pxPeripheralControl );
		pxRxTransferControlStructs[ cPeripheralNumber  ] = diGET_RX_TRANSFER_STRUCT( pxPeripheralControl );

		switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
		{
			case ioctlUSE_BLOCKED_RX :

				#if ioconfigUSE_SPI_BLOCKED_RX == 1
				{
					Blocked_TxRx_State_t *pxBlockedRxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
					HAL_StatusTypeDef rslt;
					if (pxBlockedRxState->xMode & (ioctlRX_USE_DMA_BIT | ioctlRX_USE_INT_BIT))
					{
						configASSERT( xSPI_TX_CMPL_Notify[cPeripheralNumber] == NULL );

						/* Store the handle of the calling task. */
						xSPI_RX_CMPL_Notify[cPeripheralNumber] = xTaskGetCurrentTaskHandle();

						if (pxBlockedRxState->xMode & ioctlRX_USE_DMA_BIT)
						{
							rslt = HAL_SPI_Receive_DMA(pxSPI, (uint8_t *)pvBuffer, xBytes);
						}
						else
						{
							rslt = HAL_SPI_Receive_IT(pxSPI, (uint8_t *)pvBuffer, xBytes);
						}
						if (HAL_OK == rslt)
						{
							if (ulTaskNotifyTake( pdTRUE, pxBlockedRxState->xBlockTime))
							{
								xReturn = xBytes;
							}
							else
							{
							  xSPI_RX_CMPL_Notify[cPeripheralNumber] = NULL;
							}
						}
					}
					else
					{
						rslt = HAL_SPI_Receive(pxSPI, (uint8_t *)pvBuffer, xBytes, osWaitForever);
						if (HAL_OK == rslt)
						{
							xReturn = xBytes;
						}
						else if (HAL_TIMEOUT == rslt)
						{
							xReturn = pxSPI->RxXferSize - pxSPI->RxXferCount;
						}
					}
				}
		    #endif /* ioconfigUSE_SPI_BLOCKED_RX */
				break;


			case ioctlUSE_CIRCULAR_BUFFER_RX :

				#if ioconfigUSE_SPI_CIRCULAR_BUFFER_RX == 1
				{
					Circular_Buffer_Rx_State_t *pxRxState = ( Circular_Buffer_Rx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
					TickType_t ticks = pxRxState->xBlockTime;
					uint16_t remain;
					uint16_t uart_rx_head_idx;
					uint8_t *target = pvBuffer;
					for (size_t i = 0u; i < xBytes; ++i)
					{
	          remain = __HAL_DMA_GET_COUNTER(pxSPI->hdmarx);
            uart_rx_head_idx = pxRxState->usBufferLength-remain;
						while (uart_rx_head_idx == pxRxState->usNextReadIndex && ticks > 0u)
						{
							vTaskDelay(1);
							--ticks;
							remain = __HAL_DMA_GET_COUNTER(pxSPI->hdmarx);
							uart_rx_head_idx = pxRxState->usBufferLength-remain;
						}
						if (uart_rx_head_idx == pxRxState->usNextReadIndex)
						{
						  break;
						}
					  printf("read%2d %02X\r\n", xReturn, pxRxState->pucBufferStart[pxRxState->usNextReadIndex]);
						target[xReturn++] = pxRxState->pucBufferStart[pxRxState->usNextReadIndex++];
						if (pxRxState->usNextReadIndex >= pxRxState->usBufferLength)
						{
							pxRxState->usNextReadIndex = 0u;
						}
					}
				}
				#endif /* ioconfigUSE_SPI_CIRCULAR_BUFFER_RX */
				break;


			case ioctlUSE_CHARACTER_QUEUE_RX :

				#if ioconfigUSE_SPI_RX_CHAR_QUEUE == 1
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
				#endif /* ioconfigUSE_SPI_RX_CHAR_QUEUE */
				break;


			default :

				/* Other methods can be implemented here. */
				configASSERT( xReturn );
				pxTxTransferControlStructs[ cPeripheralNumber ] = NULL;

				/* Prevent compiler warnings when the configuration is set such
				that the following parameters are not used. */
				( void ) pvBuffer;
				( void ) xBytes;
				( void ) pxSPI;
				break;
		}
	}

	return xReturn;
}

/*-----------------------------------------------------------*/

portBASE_TYPE FreeRTOS_SPI_ioctl( Peripheral_Descriptor_t pxPeripheral, uint32_t ulRequest, void *pvValue )
{
Peripheral_Control_t * const pxPeripheralControl = ( Peripheral_Control_t * const ) pxPeripheral;
uint32_t ulValue = ( uint32_t ) pvValue;
const int8_t cPeripheralNumber = diGET_PERIPHERAL_NUMBER( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
SPI_DeviceTypeDef * pxSPI = ( SPI_DeviceTypeDef * ) diGET_PERIPHERAL_BASE_ADDRESS( ( ( Peripheral_Control_t * const ) pxPeripheral ) );
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

							#if ioconfigUSE_SPI_BLOCKED_TX == 1
							{
								Blocked_TxRx_State_t *pxPlledTxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
								pxPlledTxState->xMode = ulValue;
							}
							#endif /* ioconfigUSE_SPI_BLOCKED_TX */
							break;
						case ioctlUSE_ZERO_COPY_TX :

							#if ioconfigUSE_ZERO_COPY_TX == 1
							{
								Zero_Copy_Tx_State_t *pxZeroCopyTxState = ( Zero_Copy_Tx_State_t * )pxPeripheralControl->pxTxControl->pvTransferState;
								pxZeroCopyTxState->xMode = ulValue;
							}
							#endif /* ioconfigUSE_SPI_BLOCKED_TX */
							break;
					}	
			  }
				if (diGET_RX_TRANSFER_STRUCT( pxPeripheralControl ))
				{
					switch( diGET_RX_TRANSFER_TYPE( pxPeripheralControl ) )
					{
						case ioctlUSE_BLOCKED_RX :

							#if ioconfigUSE_SPI_BLOCKED_RX == 1
							{
								Blocked_TxRx_State_t *pxPlledRxState = ( Blocked_TxRx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
								pxPlledRxState->xMode = ulValue;
							}
							#endif /* ioconfigUSE_SPI_BLOCKED_RX */
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

							#if ioconfigUSE_SPI_BLOCKED_RX == 1
							{
									__HAL_SPI_CLEAR_FLAG(pxSPI, SPI_FLAG_RXNE);
							}
							#endif /* ioconfigUSE_SPI_BLOCKED_RX */
							break;
													
						case ioctlUSE_CIRCULAR_BUFFER_RX :
							#if ioconfigUSE_SPI_CIRCULAR_BUFFER_RX == 1
							{
								Circular_Buffer_Rx_State_t *pxRxState = ( Circular_Buffer_Rx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
	              uint16_t remain = __HAL_DMA_GET_COUNTER(pxSPI->hdmarx);
                uint16_t uart_rx_head_idx = pxRxState->usBufferLength-remain;
								pxRxState->usNextReadIndex = uart_rx_head_idx;
							}
							#endif /* ioconfigUSE_SPI_CIRCULAR_BUFFER_RX */
							break;
					}	
				}
				else
				{
				  //__HAL_SPI_CLEAR_FLAG(pxSPI, SPI_FLAG_RXNE);
				}
				break;

				case ioctlSTART_CIRCULAR_BUFFER_RX_DMA:
				{
						#if ioconfigUSE_SPI_CIRCULAR_BUFFER_RX == 1
						Circular_Buffer_Rx_State_t *pxRxState = ( Circular_Buffer_Rx_State_t * )pxPeripheralControl->pxRxControl->pvTransferState;
				    HAL_SPI_Receive_DMA(pxSPI, pxRxState->pucBufferStart, pxRxState->usBufferLength);
						#endif /* ioconfigUSE_SPI_CIRCULAR_BUFFER_RX */
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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  TaskHandle_t cur_task;
	xSemaphoreHandle cur_sig;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t idx;

	idx = boardNUM_SPIS;

	if (hspi == &hspi1)
	{
		idx = 1;
	}
	else if (hspi == &hspi2)
	{
		idx = 2;
	}
	if (idx < boardNUM_SPIS)
	{
	  cur_task = xSPI_TXRX_CMPL_Notify[idx];
		cur_sig = xSPI_ZC_TX_SIG[idx];
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
	  xSPI_TXRX_CMPL_Notify[idx] = NULL;
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

