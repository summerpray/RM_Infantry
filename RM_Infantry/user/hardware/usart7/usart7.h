#ifndef __USART7_H
#define __USART7_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stm32f4xx.h"

#define EN_USART7_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	
#define VISION_BUFFER_LEN   	100    //�����˲���ϵͳ���ݽ��ճ���



void usart7_init(u32 bound);
void UART7_SendChar(uint8_t cData);//����һ���ֽڵ�����
#endif
