#include "gimbal_key_control.h"
#include "kalman.h"
#include "kalman_filter.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "vision.h"
#include "magazine.h"
#include "chassis_task.h"

#include "pid.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "user_lib.h"

#include "remote_control.h"
#include "CAN_Receive.h"


extern  RC_ctrl_t rc_ctrl;    //����ң�����ṹ�����
extern GimbalCtrlMode  modeGimbal;   //������̨����ģʽ    ��е/������
extern eGimbalAction  actGimbal;     //������̨�˶�ģʽ  ��ͷ ���� �����


//��ͷģʽ�Ƕ�Ŀ��
float TURNMode_Yaw_Back_Total;//����C,yaw��Ҫ�ı�ĽǶ�ֵ
float TURNMode_Yaw_Turn_Total;//����QE,yaw��Ҫ�ı�ĽǶ�ֵ,������������ת

float Cloud_Angle_Target_GD[2][2];   //����Ŀ��Ƕȹ��ȱ���   yaw/pitch    mech/gyro


/**
  * @brief  ��̨����ģʽѡ��,������Ӧ
  * @param  void
  * @retval void
  * @attention ��̨���̿���״̬�µ�����ģʽ�л�������
  * ��ģʽ�л�ʱһֱ���ڴ�ģʽ
  */
	
void GIMBAL_NORMAL_Mode_Ctrl(void)
{
	//������ʱ��Ӧ,��ֹ�ּ���
	static portTickType  Key_Ctrl_CurrentTime = 0;
	static uint32_t PressV_Time  = 0;//��ͷ,500ms��ʱ��Ӧ,1����ఴ2��
	static uint32_t PressQ_Time  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
  static uint32_t PressE_Time  = 0;//90��,250ms��ʱ��Ӧ,1����ఴ4��
//	static uint32_t PressCF_Time  = 0;//����,400ms��ʱ��Ӧ
//	static uint32_t PressCV_Time  = 0;//��С��,400ms��ʱ��Ӧ
	
	Key_Ctrl_CurrentTime = xTaskGetTickCount( );//��ȡʵʱʱ��,������������ʱ�ж�	
	
	
	if ( CHASSIS_IfActiveMode() == TRUE || Magazine_IfOpen() ==	 TRUE)//��ȡ����ģʽ,trueΪ��еģʽ
	{
		modeGimbal = CLOUD_MECH_MODE;
	} 
	else					//ע�͵�loop�еĵ��̻���������ģʽʧЧ
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}
	
	if ( !IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V && Key_Ctrl_CurrentTime > PressV_Time)  //Ctrl�����ڰ���״̬ʱ��V��ͷ
	{   
		actGimbal  =  GIMBAL_AROUND;//�л��ɵ�ͷģʽ

		PressV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms��ʱ���ּ���

		if(IF_KEY_PRESSED_A)//AV���ͷ
		{
			TURNMode_Yaw_Back_Total = PI;
		}
		else if(IF_KEY_PRESSED_D)//DV�ҵ�ͷ
		{
			TURNMode_Yaw_Back_Total = -PI;
		}
		else//Ĭ���ҵ�ͷ
		{
			TURNMode_Yaw_Back_Total = -PI;
		}
	}
	/*---------------------------------*/	
	else if ( !IF_KEY_PRESSED_CTRL && ( (IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > PressQ_Time) || (IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > PressE_Time) ) ) //Ctrl�����ڰ���״̬ʱ��Q(��),E(��)90���ͷ
	{   
		actGimbal = GIMBAL_TURN;//�л��ɿ���Ťͷģʽ
		
		//ע�ⷽ��
		if ( IF_KEY_PRESSED_Q)
		{
			PressQ_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms��ʱ���ּ���
			
			TURNMode_Yaw_Turn_Total = PI/2;
		}
		else if (IF_KEY_PRESSED_E)
		{
			PressE_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms��ʱ���ּ���
			
			TURNMode_Yaw_Turn_Total = -PI/2;
		}
			
	}	
	/*---------------------------------*/
	else if ( Magazine_IfWait() == TRUE || Magazine_IfOpen() == TRUE )			//���ֿ��������ڿ���,��̨���в�����

	{
		
		actGimbal = GIMBAL_LEVEL;

	}
	else if (IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)//��SW1������,���Ҽ�����
	{
		actGimbal = GIMBAL_AUTO;

	}
//	/*----------------С��-----------------*/
//	else if(IF_KEY_PRESSED_V && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCV_Time)//Ctrl+V���,400ms��Ӧһ��
//	{
//		PressCV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_SM_BUFF;
//	}
//	/*----------------���-----------------*/
//	else if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCF_Time)//Ctrl+F���,400ms��Ӧһ��
//	{
//		PressCF_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_BUFF;
//	}
//	/*----------------����-----------------*/
//	else if(IF_KEY_PRESSED_C && !IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)
//	{
//		actGimbal = GIMBAL_BASE;
//	}
	else
	{
		GIMBAL_Set_Key_Control();   //��ͨģʽ�µ�Ŀ��Ƕȼ���
	}
}	

/**
  * @brief  ����ģʽ
  * @param  void
  * @retval void
  * @attention ��ģʽ�½�ֹ����pitch
  */
void GIMBAL_LEVEL_Mode_Ctrl(void)
{
	modeGimbal = CLOUD_MECH_MODE;//����ʱ�����еģʽ
	
	//�������,�˳�����ģʽ
	if( Magazine_IfWait() == FALSE	&& Magazine_IfOpen() == FALSE )
	{
		actGimbal = GIMBAL_NORMAL;
	}
	else//����δ���,�Ƕȹ̶����м�
	{
		Cloud_Angle_Target_GD[YAW][MECH]   = mid_yaw_angle;
		Cloud_Angle_Target_GD[PITCH][MECH] = mid_pitch_angle;
	}
}
