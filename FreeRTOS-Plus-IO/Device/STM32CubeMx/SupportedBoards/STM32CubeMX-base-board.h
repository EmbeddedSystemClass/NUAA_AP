/**
 *********************************************************************
 * Description           :This file provides code for CubeMX Peripheral
 *********************************************************************
 * 
 * Copyright (c) 2017 Prince An (www.740129489@qq.com)
 * All rights reserved.
 *
 *********************************************************************
 */

#ifndef STM32CUBEMX_BASE_BOARD_H
#define STM32CUBEMX_BASE_BOARD_H

/* Header files for all the driver libraries that can be used with this BSP. */
#include "usart.h"
#include "spi.h"
#include "can.h"

/*******************************************************************************
 * Definitions used by FreeRTOS+IO to determine the peripherals that are
 * available on the board, and the functions used to interface with the target
 * specific peripheral drivers.
 ******************************************************************************/

/*******************************************************************************
 * Definitions used by the UART-interrupt-driven-command-console.c example file.
 *
 * See http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_IO/Board_Support_Packages.shtml
 *
 ******************************************************************************/
#define boardAVAILABLE_DEVICES_LIST												\
{																				\
	{ ( const int8_t * const ) "/UART1/", eUART_TYPE, ( void * ) &huart1 },	\
	{ ( const int8_t * const ) "/UART2/", eUART_TYPE, ( void * ) &huart2 },	\
	{ ( const int8_t * const ) "/UART3/", eUART_TYPE, ( void * ) &huart3 },	\
	{ ( const int8_t * const ) "/UART4/", eUART_TYPE, ( void * ) &huart4 },	\
	{ ( const int8_t * const ) "/UART5/", eUART_TYPE, ( void * ) &huart5 },	\
	{ ( const int8_t * const ) "/UART6/", eUART_TYPE, ( void * ) &huart6 },	\
	{ ( const int8_t * const ) "/CAN1/", eCAN_TYPE, ( void * ) &hcan1 },	\
	{ ( const int8_t * const ) "/CAN2/", eCAN_TYPE, ( void * ) &hcan2 },	\
	{ ( const int8_t * const ) "/SPI1/", eSPI_TYPE, ( void * ) &hspi_sfm },		\
	{ ( const int8_t * const ) "/SPI2/", eSPI_TYPE, ( void * ) &hspi_mpu }   	\
}

/*******************************************************************************
 * Map the FreeRTOS+IO interface to the Stm32CubeMX specific functions.
 ******************************************************************************/
portBASE_TYPE vFreeRTOS_stm32cubemx_PopulateFunctionPointers( const Peripheral_Types_t ePeripheralType, Peripheral_Control_t * const pxPeripheralControl );
#define boardFreeRTOS_PopulateFunctionPointers vFreeRTOS_stm32cubemx_PopulateFunctionPointers

/*******************************************************************************
 * These define the number of peripherals available on the microcontroller -
 * not the number of peripherals that are supported by the software
 ******************************************************************************/
#define boardNUM_SPIS				3 /* SPI0 to SPI2. */
#define boardNUM_UARTS			6 /* UART0 to UART5. */
#define boardNUM_I2CS				3 /* I2C0 to I2C2. */
#define boardNUM_CANS           2 /* CAN1 to CAN2 */


/*******************************************************************************
 * Command console definitions.
 ******************************************************************************/
#define boardCOMMAND_CONSOLE_UART	( const int8_t * const ) "/UART2/"

/*******************************************************************************
 * GPIO/LED polarities
 ******************************************************************************/
#define boardGPIO_OUTPUT			( 1 )
#define boardGPIO_INPUT				( 0 )
#define boardLED_ON					( 1 )
#define boardLED_OFF				( 0 )

/* SPI specific ioctl requests. */
#define ioctlSET_SPI_FRAME_FORMAT			1000

#endif /* STM32CUBEMX_BASE_BOARD_H */




