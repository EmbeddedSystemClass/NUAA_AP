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

#ifndef CIRCULAR_BUFFER_RX_H
#define CIRCULAR_BUFFER_RX_H

/* The transfer structure used when a circular buffer is used for
reception. */
typedef struct xCIRCULAR_BUFFER_RX_STATE
{
	uint8_t *pucBufferStart;			/* The start address of the buffer storage area. */
	uint16_t usBufferLength;			/* The total length of the circular buffer. */
	uint16_t usNextReadIndex;			/* Index into pucBufferStart from which the next byte will be read. */
	portTickType xBlockTime;			/* The amount of time a task should be held in the Blocked state (not using CPU time) to wait for data to become available when it attempts a read. */
} Circular_Buffer_Rx_State_t;


/* Prototypes of functions that are for internal use only. */
portBASE_TYPE xIOUtilsConfigureCircularBufferRx( Peripheral_Control_t * const pxPeripheralControl, const portBASE_TYPE xBufferSize );
void vIOUtilsSetCircularBufferTimeout( Peripheral_Control_t *pxPeripheralControl, portTickType xMaxWaitTime );

#endif /* CIRCULAR_BUFFER_RX_H */




