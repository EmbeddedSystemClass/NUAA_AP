#include "can_identifer.h"
#include "FreeRTOS.h"

#include "stm32f4xx_hal.h"
#include "can.h"
#include "structPara.h"

/*中断接收*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    
}

/*接收 解析*/
void canSbusDataParse(CanRxMsgTypeDef rx)
{
    if(rx.StdId & canMegFilter == canSbusIn)
    {
        switch(rx.StdId & canIxdFilter)
        {
            case canMegIxd0 :
                for (uint8_t i = 0; i < 4; i++)
                {
                    sBusData.channel[i] = (uint32_t)rx.Data[2*i] << 8 | rx.Data[2*i + 1];     
                }
                sBusData.update = 1;
                break;
            case canMegIxd1 :
                for (uint8_t i = 0; i < 4; i++)
                {
                    sBusData.channel[i + 4] = (uint32_t)rx.Data[2*i] << 8 | rx.Data[2*i + 1];
                }
                sBusData.update = 1;
                break;
            default :
                sBusData.update = 0;
                break;
        }
    }
}

void canAirSpeedParse(CanRxMsgTypeDef rx)
{
    if(rx.StdId & canMegFilter == canAirspeed)
    {
        switch(rx.StdId & canIxdFilter)
        {
            case canMegIxd0 :
                memcpy(&airSpeed.speed,&rx.Data[0],sizeof(float));
                memcpy(&airSpeed.pressureDiff,&rx.Data[4],sizeof(float));
                airSpeed.update = 1;
                break;
            case canMegIxd1 :
                memcpy(&airSpeed.rtSpeed,&rx.Data[0],sizeof(float));
                airSpeed.update = 1;
                break;
            default :
                airSpeed.update = 0;
                break;
        }
    }
    else
    {
        airSpeed.update = 0;
    }
}

HAL_StatusTypeDef txDataWrite(CAN_HandleTypeDef *hcan, uint32_t cnaId, uint8_t len,uint8_t *data)
{
    HAL_StatusTypeDef result = HAL_OK;

    hcan->pTxMsg->StdId = StdId;   //标准型
	hcan->pTxMsg->IDE   = CAN_ID_STD;  
	hcan->pTxMsg->RTR   = CAN_RTR_DATA; //数据帧

    if( len > 8 )
	{
		hcan->pTxMsg->DLC = 8; 
	}
	else
	{
		hcan->pTxMsg->DLC = len; 
	}

    for(uint8_t i = 0; i < hcan->pTxMsg->DLC; i++ )
	{
		hcan->pTxMsg->Data[i] = data[i];
	}

    result = HAL_CAN_Transmit(hcan,100);
    // result = HAL_CAN_Transmit_IT(hcan);
    if( result != HAL_OK )
    {
        SEGGER_RTT_printf(0,"HAL_CAN_Transmit_IT ERROR at %d\r\n", result);
    }	

    return result;
}

HAL_StatusTypeDef txDataSend(CAN_HandleTypeDef *hcan, uint32_t message, uint8_t len, uint8_t* buff)
{
    uint8_t lenTemp = len;
    uint8_t flag = 0;
    uint32_t canID ;
    while(lenTemp > 0)
    {
        len = (lenTemp > 8) ? 8 : tenTemp;
        canID = txCanIdCombine(message,flag);
        txDataWrite(&hcan, canID, len, buff);
        buff += len;
        lenTemp -= len;
        flag ++;
        if(flag > 8)
        {
            return HAL_ERROR;
        }
    }
    return HAL_OK;
}

uint32_t txCanIdCombine(uint32_t message, uint8_t flag)
{
    return message | flag ;
}