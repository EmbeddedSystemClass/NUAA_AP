/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "SEGGER_RTT.h"
#include "canParse.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId LEDTaggleTaskHandle;
osThreadId canReceiveHandle;
osThreadId canTransmiterHandle;
osSemaphoreId canDataReceiveHandle;

/* USER CODE BEGIN Variables */
extern CAN_HandleTypeDef hcan1;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartLEDTaggleTask(void const * argument);
void StartcanReceiveTask(void const * argument);
void StartTask03(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of canDataReceive */
  osSemaphoreDef(canDataReceive);
  canDataReceiveHandle = osSemaphoreCreate(osSemaphore(canDataReceive), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of LEDTaggleTask */
  osThreadDef(LEDTaggleTask, StartLEDTaggleTask, osPriorityIdle, 0, 128);
  LEDTaggleTaskHandle = osThreadCreate(osThread(LEDTaggleTask), NULL);

  /* definition and creation of canReceive */
  osThreadDef(canReceive, StartcanReceiveTask, osPriorityHigh, 0, 128);
  canReceiveHandle = osThreadCreate(osThread(canReceive), NULL);

  /* definition and creation of canTransmiter */
  osThreadDef(canTransmiter, StartTask03, osPriorityNormal, 0, 128);
  canTransmiterHandle = osThreadCreate(osThread(canTransmiter), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartLEDTaggleTask function */
void StartLEDTaggleTask(void const * argument)
{

  /* USER CODE BEGIN StartLEDTaggleTask */
  /* Infinite loop */
  for(;;)
  {
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
    osDelay(50);
  }
  /* USER CODE END StartLEDTaggleTask */
}

/* StartcanReceiveTask function */
void StartcanReceiveTask(void const * argument)
{
  /* USER CODE BEGIN StartcanReceiveTask */
	
  /* Infinite loop */
  for(;;)
  {
		CanRxMsgTypeDef rx;
		osSemaphoreWait (canDataReceiveHandle,osWaitForever);
		SEGGER_RTT_printf(0,"Can receive datas \r\n");
		rx = *hcan1.pRxMsg;
		switch (rx.StdId & canMegFilter)
		{
			case canSbusIn :
				canSbusDataParse(rx);
				SEGGER_RTT_printf(0,"Sbus at %d\r\n", sBusData.channel[0]);
				SEGGER_RTT_printf(0,"Sbus at %d\r\n", sBusData.channel[1]);
				SEGGER_RTT_printf(0,"Sbus at %d\r\n", sBusData.channel[2]);
				SEGGER_RTT_printf(0,"Sbus at %d\r\n", sBusData.channel[4]);
				break;
			case canAirspeed :
				canAirSpeedParse(rx);		                                                                                 
				break; 
			default :
				SEGGER_RTT_printf(0,"rx.StdId= %d\r\n",rx.StdId);
				SEGGER_RTT_printf(0,"Can not receive datas \r\n");
				break;
		}
    osDelay(10);
  }
  /* USER CODE END StartcanReceiveTask */
}

/* StartTask03 function */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint8_t temData[10] = {1,2,3,4,5,6,7,8,0,1};
  /* Infinite loop */
  for(;;)
  {
		hcan1.pTxMsg->StdId = (uint32_t)canSbusIn;
		if( (txDataSend(&hcan1, canSbusIn, sizeof(temData), temData)) != HAL_OK)
				SEGGER_RTT_printf(0,"canSendError\r\n");
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_9);
		temData[9]++;
    osDelay(50);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Application */
 void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
 {
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
 }
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
