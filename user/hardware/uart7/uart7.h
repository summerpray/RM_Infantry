#ifndef __UART7_H
#define __UART7_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stm32f4xx.h"

#define EN_UART7_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	
//#define JUDGE_BUF_LEN1   	200    //�����˲���ϵͳ���ݽ��ճ���

#define VISION_BUFFER_LEN 200

void uart7_init(uint32_t bound);

typedef __packed struct 
{
	uint16_t VERSION_BUFFER;

} vision_buffer_t;

void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);

void Usart_SendString( USART_TypeDef * pUSARTx, char *str);

void Uart7_SendChar(uint8_t Data);

void Usart_SendFloat( USART_TypeDef * pUSARTx, float ch);

//void Usart_Float(USART_TypeDef * pUSARTx, float data);

//void usart_send_float(USART_TypeDef * pUSARTx,float value);

#endif
