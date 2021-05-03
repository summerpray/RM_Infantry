/**
  * @file       super_cap.c/h
  * @brief      �������ݿ�������
  * @note       �ɴӳ�����ݿ��ư��ж�ȡ���ݵ�ѹ������ϵͳ�������������ϵͳ�����ѹ���趨����
								��Ҫע�⣬����Ҫ����������ʱ��Ӧ��ע���ݵ�ѹ��Ҫ����12V�������C620����ĵ�ѹ������ʹ����ϵ�
								ʹ�÷�ʽ�������̵�������أ������趨����ʱ������ϵͳ���벻�ᳬ���趨���ʣ����㲿���ɷ������ݲ���
								���ڷ��������������ޣ�����ʱ��Ҳ���ޣ����Գ�����ʱ��Ҳ����̫�á���������8F(100F,12��)
								ͨ��canͨѶ�趨���ʣ���30W-130W�ɵ���Ĭ�Ϲ���35W������ֵ3000-13000�����۸�����α仯����ϵͳ�������ʼ�ջ����130W
								���ƽ��飺
								���������趨���������Ӳ���ϵͳ��ȡ�Ĺ�������ֵ�Զ��ı䣬�����˶�����������ʵ�ֶ��ݳ�����
								
								�ر�ע�⣺ȷ������ģ��İ汾���Լ����Ӧ�Ĳ���ϵͳ����Э��汾������ᵼ���޷�����ȷ�Ĳ�����Ϣ�����³�������ʧЧ
  *
  @verbatim
  ==============================================================================
   һ����������ӽ�CAN_receive.c��
	 
	 #include "super_cap.h"
	 
	 int16_t Cap_Inputvot,Cap_Capvot,Cap_Test_current,Cap_Target_Power;
	
	 if(RxMessage.StdId == 0x211)//�������ݿ��ư�		
	{
		Cap_Inputvot  = (float)((int16_t)(RxMessage.Data[1]<<8|RxMessage.Data[0]))/100.0f;  //�����ѹ
		Cap_Update_Cap_Inputvot(Cap_Inputvot);
		
		Cap_Capvot = (float)((int16_t)(RxMessage.Data[3]<<8|RxMessage.Data[2]))/100.0f;  //���ݵ�ѹ
		Cap_Update_Cap_Capvot(Cap_Capvot);
		
		Cap_Test_current = (float)((int16_t)(RxMessage.Data[5]<<8|RxMessage.Data[4]))/100.0f;	  //�������
		Cap_Update_Cap_Test_current(Cap_Test_current);
		
		Cap_Target_Power = (float)((int16_t)(RxMessage.Data[7]<<8|RxMessage.Data[6]))/100.0f;	 //���빦��
		Cap_Update_Cap_Target_Power(Cap_Target_Power);
	}	
  ==============================================================================
  @endverbatim
  */
#include "stm32f4xx.h"
#include "super_cap.h"
#include "start_task.h"
#include "can.h"
#include "Remote_Control.h"
#include "led.h"
#include "CAN_receive.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "judge.h"

float Input_Vot;
float Supercap_Vot;
float Input_Current;
float Target_Power;

extern ext_game_robot_state_t  	GameRobotStat;//0x0201     ����������״̬


////ң�������Ʊ���
//extern RC_ctrl_t rc_ctrl;


void CAN1_Cap_Send(u16 temPower)
{	
  CanTxMsg TxMessage;
	
	TxMessage.StdId=0x210;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=0x08;
	
	TxMessage.Data[0]=(temPower >> 8);
	TxMessage.Data[1]=(temPower);
	
	CAN_Transmit(CAN1,&TxMessage);
}


void Cap_Update_Cap_Inputvot(int16_t inputvot )
{
    Input_Vot = inputvot;			//��ȡ�����ѹ
}

void Cap_Update_Cap_Capvot(int16_t capvot )
{
    Supercap_Vot = capvot;			//��ȡ���ݵ�ѹ
}

void Cap_Update_Cap_Test_current(int16_t current )
{
    Input_Current = current;			//��ȡ�������
}

void Cap_Update_Cap_Target_Power(int16_t power )
{
    Target_Power = power;			//��ȡ���빦��
}
//������
void Super_cap_task(void)
{

		portTickType currentTime;	
	
		for(;;)
		{	
			
			currentTime = xTaskGetTickCount();  	//��ǰϵͳʱ��
			
			if (SYSTEM_GetSystemState() == SYSTEM_STARTING)
			{
					Cap_Init();
			}
			
			else
			{
				if (SYSTEM_GetRemoteMode() == RC)					//ң����ģʽ
				{
					if( Supercap_Vot <= 15) //���ݵ�ѹ��Ҫ����12V������ܻ����C620��ѹ����
					{
						CAN1_Cap_Send(13000);
					}
					else
					{
						CAN1_Cap_Send(5000);
					}		
				}
				else if (SYSTEM_GetRemoteMode() == KEY)		//����ģʽ
				{
					
					if( Supercap_Vot <= 15) //���ݵ�ѹ��Ҫ����12V������ܻ����C620��ѹ����
					{
						 CAN1_Cap_Send(13000);
					}
					else if (GameRobotStat.chassis_power_limit <= 40 )//��ǰ�������������<40  Ŀ�깦��Ϊ39w
					{
						 CAN1_Cap_Send(3900);
					}
					else if(GameRobotStat.chassis_power_limit > 40 && GameRobotStat.chassis_power_limit <= 50)//��ǰ�������������40-50w  Ŀ�깦��Ϊ49w
					{
						 CAN1_Cap_Send(4900);
					}	
					else if(GameRobotStat.chassis_power_limit >50 && GameRobotStat.chassis_power_limit <= 60)//��ǰ�������������50-60w  Ŀ�깦��Ϊ59w
					{
						CAN1_Cap_Send(5900);
					}
					else if(GameRobotStat.chassis_power_limit >60 && GameRobotStat.chassis_power_limit <= 70)//��ǰ�������������60-70w  Ŀ�깦��Ϊ79w
					{
						CAN1_Cap_Send(6900);
					}
					else if(GameRobotStat.chassis_power_limit >70 && GameRobotStat.chassis_power_limit <= 80)//��ǰ�������������70-80w  Ŀ�깦��Ϊ79w
					{
						CAN1_Cap_Send(7900);
					}
					else if(GameRobotStat.chassis_power_limit >80 && GameRobotStat.chassis_power_limit <= 100)//��ǰ�������������80-100w  Ŀ�깦��Ϊ99w
					{
						CAN1_Cap_Send(9900);
					}
					else if(GameRobotStat.chassis_power_limit >100 && GameRobotStat.chassis_power_limit < 120)//��ǰ�������������100-120w  Ŀ�깦��Ϊ119w
					{
						CAN1_Cap_Send(11900);
					}
					else if(GameRobotStat.chassis_power_limit >=120)//��ǰ�������������>120w  Ŀ�깦��Ϊ130w
					{
						CAN1_Cap_Send(13000);
					}
					
				}
	    }
			
			vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);
	  }	 
}

void Cap_Init()
{
	CAN1_Cap_Send(13000);
}

/*********************************************************************************************/ //�����ڲ���

//void Cap_RC_Ctrl()
//{
//		if (rc_ctrl.rc.s[0] == RC_SW_UP)//s1������,CAN�������ݣ���������ƹ������õ�38W
//	 {
//		 CAN1_Cap_Send(3800);
//     GPIO_SetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);
//	 }
//	 
//	 if(rc_ctrl.rc.s[0] == RC_SW_MID)//s1�����м�,CAN�������ݣ���������ƹ������õ�80W
//	 {
//		 CAN1_Cap_Send(8000);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_SetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);

//	 }	
//   if(rc_ctrl.rc.s[0] == RC_SW_DOWN)//s1����,CAN�������ݣ���������ƹ������õ�100W
//	 {
//     CAN1_Cap_Send(10000);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_SetBits(GPIOG, GPIO_Pin_7);
//	 }	
//}
