/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  * @note       ���ļ�����freeRTOS����
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "CAN_Receive.h"
#include "chassis_task.h"
#include "stm32f4xx.h"
#include "rng.h"
#include "can.h"
#include "super_cap.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gimbal_task.h"
#include "Detect_Task.h"
#include "shoot_task.h"
//���̵�����ݶ�ȡ
#define get_motor_measure(ptr, rx_message)                                                     \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);     \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }

//��̨������ݶ�ȡ
#define get_gimbal_motor_measuer(ptr, rx_message)                                              \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->given_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]); \
        (ptr)->speed_rpm = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);     \
        (ptr)->temperate = (rx_message)->Data[6];                                              \
    }




static CanTxMsg GIMBAL_TxMessage;
static CanTxMsg SHOOT_TxMessage;
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
static uint8_t delay_time = 100;
#endif

/**
  * @brief  CAN1�����ж�
  * @param  void
  * @retval void
  */
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	int16_t speed_measure,rota_measure,current_measure;
  int16_t Cap_Target_Power;
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)!=RESET)
	{
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	}
	//Ħ���ֵ��ת�ٶ�ȡ,��е�Ƕ���ʱû��
	if(RxMessage.StdId == 0x205)//SHOOT_LEFT
	{
		speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		Fric_UpdateMotorSpeed(Fric_Left, speed_measure);
	}
	if(RxMessage.StdId == 0x206)//SHOOT_RIGHT
	{
		speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		Fric_UpdateMotorSpeed(Fric_Right, speed_measure);
	}
	if(RxMessage.StdId == 0x207)//REVOLVER
	{
		rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		REVOLVER_UpdateMotorAngle(rota_measure);
		
		speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		REVOLVER_UpdateMotorSpeed(speed_measure);
		
	}
	//��̨�����е�Ƕȶ�ȡ
	if(RxMessage.StdId == 0x209)//yaw����
	{
		rota_measure  = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		GIMBAL_UpdateAngle(YAW, rota_measure);
		
		speed_measure = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);        
		GIMBAL_UpdateCurrent(YAW, speed_measure);
		
		current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		GIMBAL_UpdateSpeed(YAW, current_measure);
	}
	
	if(RxMessage.StdId == 0x20A)//pitch����
	{
		rota_measure  = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
		GIMBAL_UpdateAngle(PITCH, rota_measure);
		
		speed_measure = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
		GIMBAL_UpdateCurrent(PITCH, speed_measure);
		
		current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
		GIMBAL_UpdateSpeed(PITCH, current_measure);
		
	}
	
	 if(RxMessage.StdId == 0x211)//�������ݿ��ư�		
	{
//		Cap_Inputvot  = (float)((int16_t)(RxMessage.Data[1]<<8|RxMessage.Data[0]))/100.0f;  //�����ѹ
//		
//		Cap_Capvot = (float)((int16_t)(RxMessage.Data[3]<<8|RxMessage.Data[2]))/100.0f;  //���ݵ�ѹ
//		
//		Cap_Test_current = (float)((int16_t)(RxMessage.Data[5]<<8|RxMessage.Data[4]))/100.0f;	  //�������
		
		Cap_Target_Power = (float)((int16_t)(RxMessage.Data[7]<<8|RxMessage.Data[6]))/100.0f;	 //���빦��
		Cap_UpdateTarget_Power(Cap_Target_Power);
	}	
}

void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	int16_t speed_measure,rota_measure,current_measure;
	
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) //��⵽�жϷ���
    {				 
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);//����жϹ���
		CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);

		if(RxMessage.StdId == 0x201)		//ǰ��	

		{
			rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
			CHASSIS_UpdateMotorAngle(LEFT_FRON_201, rota_measure);
		
			speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
			CHASSIS_UpdateMotorSpeed(LEFT_FRON_201, speed_measure);
		
			current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
			CHASSIS_UpdateMotorCur(LEFT_FRON_201, current_measure);
		}		
		
		if(RxMessage.StdId == 0x202)	 //ǰ��		

		{
			rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
			CHASSIS_UpdateMotorAngle(RIGH_FRON_202, rota_measure);
		
			speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
			CHASSIS_UpdateMotorSpeed(RIGH_FRON_202, speed_measure);
		
			current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
			CHASSIS_UpdateMotorCur(RIGH_FRON_202, current_measure);
		}		
		
		if(RxMessage.StdId == 0x203)			//����

		{
			rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
			CHASSIS_UpdateMotorAngle(LEFT_BACK_203, rota_measure);
		
			speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
			CHASSIS_UpdateMotorSpeed(LEFT_BACK_203, speed_measure);
		
			current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
			CHASSIS_UpdateMotorCur(LEFT_BACK_203, current_measure);
		}		
		
		if(RxMessage.StdId == 0x204)		//����	

		{
			rota_measure   = ((int16_t)RxMessage.Data[0]<<8|RxMessage.Data[1]);
			CHASSIS_UpdateMotorAngle(RIGH_BACK_204, rota_measure);

			speed_measure  = ((int16_t)RxMessage.Data[2]<<8|RxMessage.Data[3]);
			CHASSIS_UpdateMotorSpeed(RIGH_BACK_204, speed_measure);
		
			current_measure = ((int16_t)RxMessage.Data[4]<<8|RxMessage.Data[5]);
			CHASSIS_UpdateMotorCur(RIGH_BACK_204, current_measure);
		}

				
	}
}

//Chassis   0X1FF   4��3508   ��� 5 6 7 8 //CAN2
//Gimbal    0x2FF   2��6020   ��� 5 6     //CAN1
//Trigger   0X200   2��3508   ��� 1 2     //CAN1  ����Ͳ��̽����һ��
//Revolver  0X200   1��2006   ��� 3       //CAN1


void CAN_CMD_SHOOT(int16_t shoot_left, int16_t shoot_right, int16_t revolver, int16_t none)
{
	SHOOT_TxMessage.StdId =CAN_SHOOT_ALL_ID;
	SHOOT_TxMessage.IDE = CAN_ID_STD;
    SHOOT_TxMessage.RTR = CAN_RTR_DATA;
    SHOOT_TxMessage.DLC = 0x08;
    SHOOT_TxMessage.Data[0] = shoot_left >> 8;
    SHOOT_TxMessage.Data[1] = shoot_left;
    SHOOT_TxMessage.Data[2] = shoot_right >> 8;
    SHOOT_TxMessage.Data[3] = shoot_right;
    SHOOT_TxMessage.Data[4] = revolver >> 8;
    SHOOT_TxMessage.Data[5] = revolver;
    SHOOT_TxMessage.Data[6] = none >> 8;
    SHOOT_TxMessage.Data[7] = none;
	
	CAN_Transmit(SHOOT_CAN, &SHOOT_TxMessage);
}

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
void GIMBAL_lose_slove(void)
{
        delay_time = RNG_get_random_range(13,239);
}
#endif
//������̨�����������revΪ�����ֽ�
void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = 0;
    GIMBAL_TxMessage.Data[5] = 0;
    GIMBAL_TxMessage.Data[6] = 0;
    GIMBAL_TxMessage.Data[7] = 0;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE

    TIM6->CNT = 0;
    TIM6->ARR = delay_time ;

    TIM_Cmd(TIM6,ENABLE);
#else
    CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif

}

void TIM6_DAC_IRQHandler(void)
{
    if( TIM_GetITStatus( TIM6, TIM_IT_Update )!= RESET )
    {

        TIM_ClearFlag( TIM6, TIM_IT_Update );
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
        CAN_Transmit( GIMBAL_CAN,  &GIMBAL_TxMessage );
#endif
        TIM_Cmd(TIM6,DISABLE);
    }
}

//���͵��̵����������
void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg TxMessage;
    TxMessage.StdId = CAN_CHASSIS_ALL_ID;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.DLC = 0x08;
    TxMessage.Data[0] = motor1 >> 8;
    TxMessage.Data[1] = motor1;
    TxMessage.Data[2] = motor2 >> 8;
    TxMessage.Data[3] = motor2;
    TxMessage.Data[4] = motor3 >> 8;
    TxMessage.Data[5] = motor3;
    TxMessage.Data[6] = motor4 >> 8;
    TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &TxMessage);
}







