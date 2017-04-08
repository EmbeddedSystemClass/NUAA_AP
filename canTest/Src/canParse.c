
#include "canParse.h"

static uint32_t txCanIdCombine(uint32_t message, uint8_t flag);
static HAL_StatusTypeDef txDataWrite(CAN_HandleTypeDef *hcan, uint32_t canId, uint8_t len,uint8_t *data);
/*中断接收*/


/*接收 解析*/
void canSbusDataParse(CanRxMsgTypeDef rx)
{
	//SEGGER_RTT_printf(0,"flag %d\r\n", rx.StdId & canMegFilter);
    //SEGGER_RTT_printf(0,"flag %d\r\n", rx.StdId & canIxdFilter);
    switch(rx.StdId & canIxdFilter)
    {
        case canMegIxd0 :
            for (uint8_t i = 0; i < 4; i++)
            {
                sBusData.channel[i] = (uint32_t)rx.Data[2*i] << 8 | rx.Data[2*i + 1];   
                //SEGGER_RTT_printf(0,"rx.Data %d\r\n", rx.Data[i]);
                //SEGGER_RTT_printf(0,"channel %d\r\n", sBusData.channel[i]);
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

void canAirSpeedParse(CanRxMsgTypeDef rx)
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
	
    result = HAL_CAN_Transmit(hcan,100);
    //SEGGER_RTT_printf(0,"TranResult = %d\r\n",result);
		//result = HAL_CAN_Transmit_IT(hcan);
    if( result != HAL_OK )
    {
        SEGGER_RTT_printf(0,"HAL_CAN_Transmit ERROR at %d\r\n", result);
    }	

    return result;
}

HAL_StatusTypeDef txDataSend(CAN_HandleTypeDef *hcan, uint32_t message, uint8_t len, uint8_t* buff)
{
    uint8_t lenTemp = len;
    uint8_t flag = 0;
    uint32_t canID;
    HAL_StatusTypeDef flagD = HAL_OK;
    while(lenTemp > 0)
    {
        len = (lenTemp > 8) ? 8 : lenTemp;
        canID = txCanIdCombine(message, flag);
			//	SEGGER_RTT_printf(0,"message= %d\r\n",message);
			//	SEGGER_RTT_printf(0,"canID= %d\r\n",canID);
        if((flagD = txDataWrite(hcan, canID, len, buff))!= HAL_OK)
				{
					SEGGER_RTT_printf(0,"WriteError\r\n");
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

static uint32_t txCanIdCombine(uint32_t message, uint8_t flag)
{
    return message | flag;
}