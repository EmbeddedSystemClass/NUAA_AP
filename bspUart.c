#include "bspUartx.h"

/*中断环形缓冲接收，中断队列发送*/
Peripheral_Descriptor_t uartxPatternInit(UARTEnum eUart)
{
    Peripheral_Descriptor_t xOpenedPort;
    xOpenedPort = uartxInit(eUart);
    portBASE_TYPE ioc1 = pdFALSE , ioc2 = pdFALSE;
    if(xOpenedPort != NULL)
    {
        switch (eUart)
        {
            case eUART1 :
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_CIRCULAR_BUFFER_RX, (void *) UART1_CIRCULAR_BUFFER_RX);
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_RX_TIMEOUT, (void *) BLOCK_100_MS);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_TX_CHAR_QUEUE, (void *) UART1_TX_QUEUE_SIZE);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_TX_TIMEOUT, (void *) BLOCK_100_MS);
                  FreeRTOS_ioctl(xOpenedPort, ioctlUSE_MODE, (void *)(ioctlTX_USE_DMA_BIT|ioctlTX_USE_INT_BIT|ioctlRX_USE_INT_BIT));
                break;
            case eUART2 :
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_CIRCULAR_BUFFER_RX, (void *) UART2_CIRCULAR_BUFFER_RX);
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_RX_TIMEOUT, (void *) BLOCK_100_MS);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_TX_CHAR_QUEUE, (void *) UART2_TX_QUEUE_SIZE);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_TX_TIMEOUT, (void *) BLOCK_100_MS);
                  FreeRTOS_ioctl(xOpenedPort, ioctlUSE_MODE, (void *)(ioctlTX_USE_DMA_BIT|ioctlTX_USE_INT_BIT|ioctlRX_USE_INT_BIT));
                break;
            case eUART3 :
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_CIRCULAR_BUFFER_RX, (void *) UART3_CIRCULAR_BUFFER_RX);
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_RX_TIMEOUT, (void *) BLOCK_100_MS);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_TX_CHAR_QUEUE, (void *) UART3_TX_QUEUE_SIZE);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_TX_TIMEOUT, (void *) BLOCK_100_MS);
                  FreeRTOS_ioctl(xOpenedPort, ioctlUSE_MODE, (void *)(ioctlTX_USE_DMA_BIT|ioctlTX_USE_INT_BIT|ioctlRX_USE_INT_BIT));
                break;
            case eUART4 :
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_CIRCULAR_BUFFER_RX, (void *) UART4_CIRCULAR_BUFFER_RX);
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_RX_TIMEOUT, (void *) BLOCK_100_MS);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_TX_CHAR_QUEUE, (void *) UART4_TX_QUEUE_SIZE);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_TX_TIMEOUT, (void *) BLOCK_100_MS);
                  FreeRTOS_ioctl(xOpenedPort, ioctlUSE_MODE, (void *)(ioctlTX_USE_DMA_BIT|ioctlTX_USE_INT_BIT|ioctlRX_USE_INT_BIT));
                break;
            case eUART5 :
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_CIRCULAR_BUFFER_RX, (void *) UART5_CIRCULAR_BUFFER_RX);
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_RX_TIMEOUT, (void *) BLOCK_100_MS);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_TX_CHAR_QUEUE, (void *) UART5_TX_QUEUE_SIZE);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_TX_TIMEOUT, (void *) BLOCK_100_MS);
                  FreeRTOS_ioctl(xOpenedPort, ioctlUSE_MODE, (void *)(ioctlTX_USE_DMA_BIT|ioctlTX_USE_INT_BIT|ioctlRX_USE_INT_BIT));
                break;
            case eUART6 :
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_CIRCULAR_BUFFER_RX, (void *) UART6_CIRCULAR_BUFFER_RX);
                  ioc1 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_RX_TIMEOUT, (void *) BLOCK_100_MS);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioconfigUSE_UART_TX_CHAR_QUEUE, (void *) UART6_TX_QUEUE_SIZE);
                  ioc2 = FreeRTOS_ioctl(xOpenedPort, ioctlSET_TX_TIMEOUT, (void *) BLOCK_100_MS);
                  FreeRTOS_ioctl(xOpenedPort, ioctlUSE_MODE, (void *)(ioctlTX_USE_DMA_BIT|ioctlTX_USE_INT_BIT|ioctlRX_USE_INT_BIT));
                break;
            default :
                ioc1 = pdFALSE;
                ioc2 = pdFALSE;                
        }

        if((ioc1 || ioc2) == pdFALSE)
        {
            SEGGER_RTT_printf(0,"FreeRTOS_ioctl Failed \r\n");
        }
    }
    return xOpenedPort;
}

HAL_StatusTypeDef uartxQueueTransmiter(Peripheral_Descriptor_t xPort, uint8_t *buf, uint16_t size)
{
    HAL_StatusTypeDef xReturn = HAL_OK;
    size_f xBytesTransferred = 0；
    xBytesTransferred = FreeRTOS_write( xPort, buf, size );
   // configASSERT( xBytesTransferred == size );
    if(xBytesTransferred != size)
    {
        SEGGER_RTT_printf(0,"uarttxQueueSend is Failed \r\n");
        xReturn = HAL_ERROR;
    }
    return xReturn;
}

HAL_StatusTypeDef uartBufferReceive(Peripheral_Descriptor_t xPort, uint8_t *buf, uint16_t )
{
     HAL_StatusTypeDef xReturn = HAL_OK;
     size_f xBytesTransferred = 0；
     xBytesTransferred = FreeRTOS_read( xPort, buf, size );
     if(xBytesTransferred != size)
    {
        SEGGER_RTT_printf(0,"uarttxQueueSend is Failed \r\n");
        xReturn = HAL_ERROR;
    }
    return xReturn;

}

Peripheral_Descriptor_t parsePackFunction()
{
    parsePackFunStruct *parsePack = NULL;
    parsePack = pvPortMalloc(sizeof(parsePackFunStruct));
    if(parsePack != NULL)
    {
        parsePack->rxParse = NULL;
        parsePack->txPack = NULL;
    }
    return (Peripheral_Descriptor_t)pasePack;
}

Peripheral_Descriptor_t uartxInit(UARTEnum eUart)
{
    Peripheral_Descriptor_t xPort;
    switch(eUart)
    {
        case eUART1 :
            xPort = FreeRTOS_open("/UART1/", NULL);
          break;
        case eUART2 :
            xPort = FreeRTOS_open("/UART2/", NULL);
          break;
        case eUART3 :
            xPort = FreeRTOS_open("/UART3/", NULL);
          break;
        case eUART4 :
            xPort = FreeRTOS_open("/UART4/", NULL);
          break;
        case eUART5 :
            xPort = FreeRTOS_open("/UART5/",NULL);
          break;
        case eUART6 :
            xPort = FreeRTOS_open("/UART6/",NULL);
          break;
        default :
            xPort = NULL;
    }
    return xPort;
}

