#include "stm32wba55xx.h"            
#include "string.h"
#include "stdio.h"
#include "float.h"
#include "stdint.h"
#include "stm32wbaxx_hal.h"          
#include "uart.h"



UART_HandleTypeDef stLink_Uart;
void errHandler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
void uart_Init(void)
{
  stLink_Uart.Instance = USART2;
  stLink_Uart.Init.BaudRate = 115200;
  stLink_Uart.Init.WordLength = UART_WORDLENGTH_8B;
  stLink_Uart.Init.StopBits = UART_STOPBITS_1;
  stLink_Uart.Init.Parity = UART_PARITY_NONE;
  stLink_Uart.Init.Mode = UART_MODE_TX_RX;
  stLink_Uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  stLink_Uart.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&stLink_Uart) != HAL_OK)
  {
    errHandler();
  }
}

void uart_PrintString(char * str)
{
  HAL_UART_Transmit(&stLink_Uart, (unsigned char *)str, strlen(str), HAL_MAX_DELAY);

}


void uart_PrintFloat(float value)
{
    char buf[24]; 

    int integerPart = (int)value;
    int fractionalPart = (int)((value - integerPart) * 100);  
    
    if (fractionalPart < 0)
    {
        fractionalPart = -fractionalPart;
    }

    if (fractionalPart == 0)
    {
        snprintf(buf, sizeof(buf), "%d", integerPart); 
    }
    else
    {
        snprintf(buf, sizeof(buf), "%d.%02d", integerPart, fractionalPart); 
    }

    HAL_UART_Transmit(&stLink_Uart, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

void uart_PrintInt(unsigned int value, unsigned char base)
{
	uint8_t buf[12];

	switch(base){
	case 10: sprintf((char*)buf, "%u", value); break;
	case 16: sprintf((char*)buf, "%x", value); break;
	}

	HAL_UART_Transmit(&stLink_Uart, buf, strlen((char*)buf), HAL_MAX_DELAY);
}

void uart_PrintChar(char c)
{
    HAL_UART_Transmit(&stLink_Uart, (uint8_t *)&c, 1, HAL_MAX_DELAY);
}



