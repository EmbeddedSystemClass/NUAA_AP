#ifndef __BSPUARTX_H
#define __BSPUARTX_H

#include "FreeRTOS_IO.h"

#define BLOCK_100_MS 100

#define UART1_TX_QUEUE_SIZE 20 
#define UART2_TX_QUEUE_SIZE 20 
#define UART3_TX_QUEUE_SIZE 20 
#define UART4_TX_QUEUE_SIZE 20 
#define UART5_TX_QUEUE_SIZE 20
#define UART6_TX_QUEUE_SIZE 20

#define UART1_CIRCULAR_BUFFER_RX 20
#define UART2_CIRCULAR_BUFFER_RX 20
#define UART3_CIRCULAR_BUFFER_RX 20
#define UART4_CIRCULAR_BUFFER_RX 20
#define UART5_CIRCULAR_BUFFER_RX 20
#define UART6_CIRCULAR_BUFFER_RX 20

typedef enum{
    eUART1 = 0,
    eUART2,
    eUART3,
    eUART4,
    eUART5,
    eUART6
}UARTEnum;


typedef portBASE_TYPE (*uartRxParse)(uint8_t *buf, uint16_t size);
typedef portBASE_TYPE (*uartTxPack)(uint8_t *buf,uint16_t size);

typedef struct{
    uartRxParse rxParse;
    uartTxPack txPack;
}parsePackFunStruct;

extern parsePackFunStruct parsePackStruct;

Peripheral_Descriptor_t uartxInit(UARTEnum eUart);
Peripheral_Descriptor_t uartxPatternInit(UARTEnum eUart);

HAL_StatusTypeDef uartxQueueTransmiter(Peripheral_Descriptor_t xPort, uint8_t *buf, uint16_t size);
HAL_StatusTypeDef uartBufferReceive(Peripheral_Descriptor_t xPort, uint8_t *buf, uint16_t );

#endif


