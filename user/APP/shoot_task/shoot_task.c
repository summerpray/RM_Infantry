#include "shoot_task.h"
#include "can.h"
#include "fric.h"
#include "CAN_Receive.h"
#include "main.h"
#include "judge.h"
#include "remote_control.h"

#include <stdlib.h>
#include "arm_math.h"
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "start_task.h"
#include "gimbal_task.h"
#include "vision.h"
#include "magazine.h"
extern RC_ctrl_t rc_ctrl;
eRevolverCtrlMode Revolver_mode; //�ٶȻ� λ�û�
eShootAction actShoot;			 //���ģʽ   ���� ������ ����Ƶ������  ����Ƶ������ ���ģʽ �����Զ����
Fric_Motor_t Fric_Motor;		 //Ħ���ֵ��ID
extern VisionRecvData_t VisionRecvData;

/*******************Ħ���ֵ������**********************/
float Fric_Output[5] = {1000 ,2000 ,3000, 4000, 5000};
float Friction_PWM_Output[6] = {0, 300, 310, 320, 330, 340}; //�ر�  ����  ����  ����  ��  �ڱ�
uint16_t FricMode = 1;										 //Ħ����ģʽѡ��
//Ħ���ֲ�ͬpwm�¶�Ӧ����������ֵ(����),��ñ�ʵ��ֵ��5
uint16_t Friction_PWM_HeatInc[5] = {0, 20, 26, 34, 36}; //����ʱ��㶨���ٶ�,������Ը���

/*******************Ħ���ֵ������**********************/
 //Ŀ��ת��
float Fric_Speed_Target[2];         //�ݶ�����Զ �����ٵ���ƫ   ����6000    ����������ٸ���Ƶ  ����4000

//Ħ���ֲ����ٶ�
int16_t Fric_Speed_Measure[2];

//Ħ�����ٶ����
float Fric_Speed_Error[2];//ID
float Fric_Speed_Error_Sum[2];
//����PID����
float Fric_Speed_kpid[2][3];//	motorID kp/ki/kd

float pTermFric[2], iTermFric[2], dTermFric[2];//ID
float	pidTermFric[2];//ID,���������

//���̵�������
float Fric_Final_Output[2];

/*******************���̲���**********************/

//�ٶȵȼ�ѡ��
uint16_t Fric_Speed_Level;

//ң��ģʽ�µĿ�����־λ
uint8_t Friction_Switch = 0; //�뵯�ֿ����ж�����

//Ħ����Ŀ���ٶ�
float Friction_Speed_Target;

//Ħ���ֵȼ�Ŀ��ת��
float Frict_Speed_Level_Target;

//Ħ����ʵ������ٶ�,������б�����
float Friction_Speed_Real;

/*******************���̲���**********************/
//���̲����ٶ�
int16_t Revolver_Speed_Measure;

//���̲����Ƕ�
int16_t Revolver_Angle_Measure;

//�����ٶ����
float Revolver_Speed_Error;

//���̽Ƕ����
float Revolver_Angle_Error[2]; //  inner/outer

//�ۼƺ�
float Revolver_Angle_Measure_Sum;	 //���̲����Ƕ��ۼƺ�,����λ��PID
int16_t Revolver_Angle_Measure_Prev; //�ϴ�ʣ�µ��ۼӺͽǶ�,����Ȧ�������ж�

//����Ŀ��Ƕ�
float Revolver_Angle_Target;

//����Ŀ��Ƕ��ۼƺ�,����λ��PID����
float Revolver_Angle_Target_Sum;
float Revolver_Buff_Target_Sum;			   //���ģʽ�µ�Ŀ��ֵ�ݴ棬��ʱĿ��Ƕ���б�´���
float Revolver_Buff_Ramp = AN_BULLET / 40; //40msתһ��,һ�����ܳ���50ms

//����Ŀ��ת��
float Revolver_Speed_Target; //ת�ٹ������׿���,������ת����6000

//���̵�������,������ʱ��
float Revolver_Final_Output;

float shooter_heat_cooling;

/****************��Ƶ����******************/

uint16_t Fric_enable = 0;	 //����ģʽ����Ħ�����Ƿ���
uint16_t Chass_Switch_G = 0; //Ħ����״̬�ı���

#define SHOOT_LEFT_TIME_MAX 200 //��������л����

//�����ٶȻ���Ƶ
int16_t Revolver_Freq;

//λ�û�������,ʵʱ�ɱ�,��ֵԽСλ�û�������Խ��
uint32_t Shoot_Interval = 0;

//����������Ӧ��ģʽ�л�ʱ��ʱ���óɵ�ǰʱ��
uint32_t Revol_Posit_RespondTime = 0;

uint32_t FrictionOutput = 300;

////���������
//uint32_t Shoot_Buff_Interval = TIME_STAMP_400MS;

////�����Զ�����������
//uint32_t Shoot_Auto_Interval = TIME_STAMP_1000MS;

/*****************PID����*****************/
float pTermRevolSpeed, iTermRevolSpeed;
float pTermRevolAngle[2], iTermRevolAngle[2]; //  inner/outer
float Revolver_Speed_kpid[3];				  //	kp/ki/kd
float Revolver_Angle_kpid[2][3];			  //  inner/outer    kp/ki/kd

/**********�޷�*************/

float Revolver_Output_Max; //��������޷�
float iTermRevolSpeedMax;  //�ٶȻ�΢���޷�
float iTermRevolPosiMax;   //λ�û�΢���޷�

float Fric_Final_Output_Max; //Ħ������������޷�
float iTermFricSpeedMax;	 //Ħ�����ٶȻ������޷�

/********���**********/
//�����ӵ���,��һ�¼�һ��,��һ�ż�һ��
int16_t Key_ShootNum;		 //����������
int16_t ShootNum_Allow = 0;	 //��ʣ���ŵ����Դ�
uint16_t Residue_Heat;		 //ʣ���������,�������ƿ���
uint16_t Shoot_HeatLimit;	 //��ǰ�ȼ������������
uint16_t Shoot_HeatIncSpeed; //��ǰĦ����ת���µ����ӵ���������ֵ

/************����************/
#define Stuck_Revol_PIDTerm 4000 //PID����������������Ϊ�п��ܿ���
#define Stuck_Speed_Low 60		 //�����ٶȵ��������,����Ϊ�п��ܿ���

#define Stuck_SpeedPID_Time 100 //�ٶ����� ms��С,PID����  ms����
#define Stuck_TurnBack_Time 100 //��תʱ��,ʱ��Խ������Խ��
uint32_t Stuck_Speed_Sum = 0;	//���㿨������,�ٶȻ�
uint32_t Stuck_Posit_Sum = 0;	//���㿨������,λ�û�

portTickType posishoot_time; //�����ʱ����

/*����*/
uint8_t Revol_Switch_Left = 0;
u8 Revol_Key_Left_Change = 0;

uint8_t First_Into_Buff = FALSE;
uint8_t Buff_Shoot_Begin = FALSE;
bool buff_fire = 0;
bool buff_change_fire = 0;
bool Shoot_Switch_R = 0;
uint16_t measure_first; //������
/************************************************************************************/

uint8_t revol_remot_change = TRUE;
void shoot_task(void *pvParameters)
{
	//portTickType currentTime;

	for (;;)
	{
		//currentTime = xTaskGetTickCount();//��ǰϵͳʱ��

		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)
		{
			REVOLVER_Rest();
			SHOOT_InitArgument();
			measure_first = Revolver_Angle_Measure;
		}
		else
		{
			if (IF_RC_SW2_UP)
			{
				REVOLVER_Key_Ctrl();
				revol_remot_change = TRUE;
				Fric_Power_Change();
				Fric_Key_Ctrl();
				Magazine_Ctrl();
			}
			else
			{
				REVOLVER_Rc_Ctrl();
			}

			//����ƽ̨    ��������������Ҫȥ�����������ע��
			//			if(IF_RC_SW1_DOWN)
			//			{
			//				REVOLVER_SHAKE();
			//				Revolver_mode = REVOL_POSI_MODE;
			//			}
		}
		//----------------------------------------------
		if (Revolver_mode == REVOL_SPEED_MODE)
		{
			REVOL_SpeedLoop();
		}
		else if (Revolver_mode == REVOL_POSI_MODE)
		{
			REVOL_PositionLoop();
		}

		/*��������		 
		if(	Fric_Speed_Target[Fric_Left]>10&&	Fric_Speed_Target[Fric_Right]>10)
		{
��������*/
		Fric_Speed_PID_Calculate();
		SHOOT_CANbusCtrlMotor();
		/*��������		
		}
		else
		{
			Revolver_Speed_Target = 0;//Ħ���ֹر�,���̲�����
			Revolver_Angle_Rest();//Ħ���ֹرգ��������ڼ�Ĵ�ָ��
			REVOL_SpeedLoop();
      Reset_Fric();
			REVOLVER_CANbusCtrlMotor();
		}
��������*/
		vTaskDelay(2);
	}
}

/**
  * @brief  ����ģʽĦ���ָı�״̬
  * @param  void
  * @retval void
  * @attention
  */
void Fric_Power_Change(void)
{
	portTickType ulTimeCurrent = 0;
	static uint32_t Last_Press_R = 0;
	static uint32_t Last_Press_B = 0;
	ulTimeCurrent = xTaskGetTickCount();

	if (IF_KEY_PRESSED_R || IF_KEY_PRESSED_B)
	{
		if (IF_KEY_PRESSED_R)
		{
			if (ulTimeCurrent - Last_Press_R > TIME_STAMP_500MS)
			{
				FricMode += 1;
			}
			Last_Press_R = xTaskGetTickCount();
		}
		else if (IF_KEY_PRESSED_B)
		{
			if (ulTimeCurrent - Last_Press_B > TIME_STAMP_500MS)
			{
				FricMode -= 1;
			}
			Last_Press_B = xTaskGetTickCount();
		}
		if (FricMode >=5)
		{
			FricMode = 5;
		}
		if (FricMode <=1)
		{
			FricMode = 1;
		}
	}
}

/**
  * @brief  ����ģʽĦ���ָı�״̬
  * @param  void
  * @retval void
  * @attention
  */
void Fric_Key_Ctrl(void)
{

	if (!IF_KEY_PRESSED_G)
	{
		Chass_Switch_G = 1;
	}

	if (IF_KEY_PRESSED_G && Chass_Switch_G == 1)
	{
		Chass_Switch_G = 0;
		Fric_enable++;
		Fric_enable %= 2; //����������Ч��ż������Ч����һ�ο��ٰ�һ�ι�
	}
	if (Fric_enable)
	{
		Set_Fric_Speed(1);
		Fric_Open(Friction_PWM_Output[FricMode], Friction_PWM_Output[FricMode]);
	}
	else
	{
		Fric_Open(Friction_PWM_Output[FRI_OFF], Friction_PWM_Output[FRI_OFF]);
		Reset_Fric();

		Fric_Speed_Target[Fric_Left] = 0;
	  	Fric_Speed_Target[Fric_Right] = 0;
	}
}

/**
  * @brief  Ħ���ֵ��PID����,����
  * @param  ���ID
  * @retval void
  * @attention  (Fric_Left,Fric_Right --> ��Ħ���֣���Ħ����)
  */

void Fric_SpeedLoop(Fric_Motor_t Fric_Motor)
{
  	Fric_Speed_Error[Fric_Motor] =Fric_Speed_Target[Fric_Motor]-Fric_Speed_Measure[Fric_Motor];
	  Fric_Speed_Error_Sum[Fric_Motor] +=Fric_Speed_Error[Fric_Motor];
	  pTermFric[Fric_Motor]  =Fric_Speed_Error[Fric_Motor]*Fric_Speed_kpid[Fric_Motor][KP];
	  iTermFric[Fric_Motor] +=Fric_Speed_Error[Fric_Motor]*Fric_Speed_kpid[Fric_Motor][KI];
	
	  //�����޷�
	  iTermFric[Fric_Motor] =constrain_float(iTermFric[Fric_Motor],-iTermFricSpeedMax,iTermFricSpeedMax);

	  pidTermFric[Fric_Motor] =pTermFric[Fric_Motor]+iTermFric[Fric_Motor];
	
	  pidTermFric[Fric_Motor] =constrain_float(pidTermFric[Fric_Motor],-Fric_Final_Output_Max,Fric_Final_Output_Max);
	  
	  Fric_Final_Output[Fric_Motor] = pidTermFric[Fric_Motor];
}

/**
 * @brief �������PID
 * 
 */
void Fric_Speed_PID_Calculate(void) //���տ��Ƶ���ֵ����
{
    Fric_SpeedLoop(Fric_Left);
	Fric_SpeedLoop(Fric_Right);
}

/**
  * @brief  ��ȡ���ת��
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202 --> ��Ħ���֣���Ħ����),CAN1�е���
  */

void Fric_UpdateMotorSpeed( Fric_Motor_t Fric_Motor, int16_t speed_rev )
{
	Fric_Speed_Measure[Fric_Motor] = speed_rev;
}

/**
  * @brief  ����ģʽĦ���ָı�״̬
  * @param  void
  * @retval void
  * @attention
  */
void Fric_RC_Ctrl(void)
{
	if (IF_RC_SW1_DOWN)
	{
		Fric_Open(Friction_PWM_Output[FRI_LOW], Friction_PWM_Output[FRI_LOW]);
	}
	else if (IF_RC_SW1_UP)
	{
		Reset_Fric();
	}
}

/**
  * @brief  ����Ħ���ֵ������ٸ���
  * @param  void
  * @retval void
  * @attention 
  */
void Set_Fric_Speed(int8_t speed_mode)
{
	if (speed_mode == 0)
	{
		Fric_Speed_Target[Fric_Left]=-1750;
	    Fric_Speed_Target[Fric_Right]=1750;
	}
	else if(speed_mode == 1)
	{
		Fric_Speed_Target[Fric_Left]=-4100;
	    Fric_Speed_Target[Fric_Right]=4100;
	}
	else if(speed_mode == 2)
	{
		Fric_Speed_Target[Fric_Left]=-6000;
	    Fric_Speed_Target[Fric_Right]=6000;
	}
}

/**
  * @brief  ���̲�����ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void SHOOT_InitArgument(void)
{
	/* Ŀ��ֵ */
	Revolver_Final_Output = 0;
	Revolver_Speed_Target = 0;

	/* PID���� */
	//�ٶȻ�
	Revolver_Speed_kpid[KP] = 15;
	Revolver_Speed_kpid[KI] = 0;
	Revolver_Speed_kpid[KD] = 0;
	//λ�û�
	Revolver_Angle_kpid[OUTER][KP] = 0.4;
	Revolver_Angle_kpid[OUTER][KI] = 0;
	Revolver_Angle_kpid[OUTER][KD] = 0;
	Revolver_Angle_kpid[INNER][KP] = 6;
	Revolver_Angle_kpid[INNER][KI] = 0;
	Revolver_Angle_kpid[INNER][KD] = 0;

	/* �޷� */
	iTermRevolSpeedMax = 250;
	iTermRevolPosiMax = 2500;
	Revolver_Output_Max = 9999;

	iTermFricSpeedMax = 3000;
	Fric_Final_Output_Max = 9000;

	/* ��� */
	Key_ShootNum = 0;
	Shoot_HeatLimit = 240; //������ʼ��
	Revolver_Freq = 0;	   //��Ƶ��ʼ��

	/* λ�û�Ŀ��Ƕ� */
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure; //������0,�����ϵ�ᷴת
	Revolver_Buff_Target_Sum = Revolver_Angle_Measure;

	//Ħ�����ٶȻ�
	//��Ħ����
	Fric_Speed_kpid[Fric_Left][KP] =2;
    Fric_Speed_kpid[Fric_Left][KI] =0.5;
	Fric_Speed_kpid[Fric_Left][KD] =0;
	//��Ħ����
	Fric_Speed_kpid[Fric_Right][KP] =2;
	Fric_Speed_kpid[Fric_Right][KI] =0.5;
	Fric_Speed_kpid[Fric_Right][KD] =0;
	
	iTermFricSpeedMax = 3000;								//Ħ�����ٶȻ������޷�
	  
	Fric_Final_Output_Max =9000;						//Ħ������������޷�

}

/**
  * @brief  ���̺�Ħ��������
  * @param  void
  * @retval void
  * @attention ǹ�ڳ���������
  */
void REVOLVER_Rest(void)
{
	Key_ShootNum = 0;		   //λ�û���������
	Revolver_Speed_Target = 0; //�ٶȻ�ֹͣת��

	//�ٶȻ�λ������
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure;	  //λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum = Revolver_Angle_Measure;  //λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure; //�ϴ�λ������
	Revolver_Buff_Target_Sum = Revolver_Angle_Measure;

	//PID��������
	iTermRevolSpeed = 0;
	iTermRevolAngle[INNER] = 0;
}

/**
  * @brief  ���̽Ƕ�����
  * @param  void
  * @retval void
  * @attention ģʽ�л�ʱ��,��ֹ�´��л�ȥ��ͻȻ��һ��
  */
void Revolver_Angle_Rest(void)
{
	Key_ShootNum = 0;
	Revolver_Angle_Target_Sum = Revolver_Angle_Measure;	  //λ�û�Ŀ��Ƕ�����
	Revolver_Angle_Measure_Sum = Revolver_Angle_Measure;  //λ�û�ת���Ƕ�����
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure; //�ϴ�λ������
	Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
}

/**
  * @brief  ���̵�ң��ģʽ
  * @param  void
  * @retval void
  * @attention ң�����ٶȻ�
  */
void REVOLVER_Rc_Ctrl(void)
{

	/*******************�����**********************/

	if (IF_RC_SW2_MID) //��еģʽ�µ�����������Ե���
	{
		Revolver_mode = REVOL_POSI_MODE; //λ�û�
		if (REVOLVER_Rc_Switch() == TRUE)
		{
			Key_ShootNum++; //��һ��
		}

		if (revol_remot_change == TRUE) //�մӼ���ģʽ�л���������շ�������
		{
			revol_remot_change = FALSE;
			Revolver_Angle_Rest(); //��ֹͻȻ�Ӽ����е�ң�ؿ�ת
		}

		if (Key_ShootNum != 0)
		{
			Key_ShootNum--;
			Revolver_Buff_Target_Sum -= AN_BULLET;
		}

		if (Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum) //����ת��ȥ
		{
			Revolver_Angle_Target_Sum = RAMP_float(Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Buff_Ramp);
		}

		if (IF_RC_SW1_UP)
		{
			Fric_Speed_Target[Fric_Left] = 0;
			Fric_Speed_Target[Fric_Right] = 0;
			Fric_mode(FRI_OFF);
			Reset_Fric();
		}
		else
		{
			
			Fric_mode(FRI_LOW);
			Set_Fric_Speed(1);
		}
		REVOL_PositStuck(); //�����жϼ���ת
	}
	/**************������*******************/
	else if (IF_RC_SW2_DOWN) //������ģʽ
	{
		Revolver_mode = REVOL_SPEED_MODE; //�ٶ�
		if (IF_RC_SW1_MID)				  //sw1�´�
		{
			Fric_Open(Friction_PWM_Output[FRI_LOW], Friction_PWM_Output[FRI_LOW]);
		}
		else if (IF_RC_SW1_DOWN)
		{
			Fric_Open(Friction_PWM_Output[FRI_LOW], Friction_PWM_Output[FRI_LOW]);
			Revolver_Freq = 5; //��Ƶѡ��
			//�ٶȻ�ת������
			Revolver_Speed_Target = -REVOL_SPEED_RATIO / REVOL_SPEED_GRID * Revolver_Freq;
		}
		else
		{
			Revolver_Speed_Target = 0;
			Revolver_Freq = 0;
			Fric_Open(Friction_PWM_Output[FRI_OFF], Friction_PWM_Output[FRI_OFF]);
		}
		REVOL_SpeedStuck(); //�����жϼ���ת
	}
	/********************************************/
}

/*******����ģʽ************/

/**
  * @brief  ���̵ļ���ģʽ
  * @param  void
  * @retval void
  * @attention ������λ�û�����
  */
void REVOLVER_Key_Ctrl(void)
{
	Revolver_mode = REVOL_POSI_MODE; //��ֹ������,Ĭ��λ�û�

	SHOOT_NORMAL_Ctrl(); //ȷ�����ģʽ

	/*- ȷ�������������ģʽ -*/
	switch (actShoot)
	{
	case SHOOT_NORMAL:
		//���ģʽѡ��,Ĭ�ϲ���
		SHOOT_NORMAL_Ctrl();
		break;

	case SHOOT_SINGLE:
		//��һ���������,��������
		SHOOT_SINGLE_Ctrl();
		break;

	case SHOOT_TRIPLE:
		//��������
		SHOOT_TRIPLE_Ctrl();
		break;

	case SHOOT_HIGHTF_LOWS:
		//B����Ƶ
		SHOOT_HIGHTF_LOWS_Ctrl();
		break;

	case SHOOT_MIDF_HIGHTS:
		//Z�Ƽ�
		SHOOT_MIDF_HIGHTS_Ctrl();
		break;

	case SHOOT_BUFF:
		//����Զ���
		Revolver_mode = REVOL_POSI_MODE;

		SHOOT_BUFF_Ctrl_Gimbal();

		break;
	}

	/*- ��ʼ����,������ -*/
	if (Revolver_mode == REVOL_SPEED_MODE) //&& Fric_GetSpeedReal() > REVOL_CAN_OPEN
	{
		REVOLVER_KeySpeedCtrl();
	}
	else if (Revolver_mode == REVOL_POSI_MODE) //&& Fric_GetSpeedReal() > REVOL_CAN_OPEN
	{
		REVOLVER_KeyPosiCtrl();
	}
}

/************************���̼���ģʽ����ģʽС����****************************/
/**
  * @brief  ����ģʽ�·���ģʽѡ��
  * @param  void
  * @retval void
  * @attention  ��ͨģʽ�������,�Ҳ�����
  */
void SHOOT_NORMAL_Ctrl(void)
{
	static uint32_t shoot_left_time = 0; //�����������ʱ��,ʱ������л�������

	/*------ ���̧�����ܴ���һ�� -------*/
	if (IF_MOUSE_PRESSED_LEFT)
	{
		if (Revol_Switch_Left == 1)
		{
			Revol_Switch_Left = 2;
		}
		else if (Revol_Switch_Left == 2)
		{
			Revol_Switch_Left = 0;
		}
	}
	else if (!IF_MOUSE_PRESSED_LEFT)
	{
		Revol_Switch_Left = 1;
		shoot_left_time = 0; //������¼�ʱ
	}
	/*------------------------------------*/

	if (IF_MOUSE_PRESSED_LEFT && shoot_left_time <= SHOOT_LEFT_TIME_MAX //�������
		&& !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		Revolver_mode = REVOL_POSI_MODE; //λ�û���
		shoot_left_time++;				 //�жϳ���,�л�
		actShoot = SHOOT_SINGLE;
	}
	else if (IF_MOUSE_PRESSED_LEFT && shoot_left_time > SHOOT_LEFT_TIME_MAX //��������200ms
			 && !IF_KEY_PRESSED_B && !IF_KEY_PRESSED_Z /*&& !GIMBAL_IfAutoHit()*/)
	{
		shoot_left_time++;
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_TRIPLE; //����ģʽ
	}
	else if (IF_KEY_PRESSED_Z //����Ƶ��������,�Ƽ�ר��
			 && !IF_MOUSE_PRESSED_LEFT && !IF_KEY_PRESSED_B)
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_MIDF_HIGHTS;
		shoot_left_time = 0;
	}
	else if (GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE) //���ģʽ�ҷ��ֶ����ģʽ
	{
		Revolver_mode = REVOL_POSI_MODE;
		actShoot = SHOOT_BUFF;
		shoot_left_time = 0;
	}

	else
	{
		actShoot = SHOOT_NORMAL;
		Shoot_Interval = 0;							   //����������
		Revol_Posit_RespondTime = xTaskGetTickCount(); //������Ӧ
		shoot_left_time = 0;
		Key_ShootNum = 0;
	}
	if (GIMBAL_IfBuffHit() == FALSE) //�˳��˴��ģʽ
	{
		First_Into_Buff = TRUE;
		Buff_Shoot_Begin = FALSE;
		buff_fire = FALSE;
	}
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_SINGLE_Ctrl(void)
{
	portTickType CurrentTime = 0;
	static uint32_t RespondTime = 0; //��Ӧ�����ʱ

	CurrentTime = xTaskGetTickCount();

	Shoot_Interval = TIME_STAMP_1000MS / 8; //���һ��8��

	if (RespondTime < CurrentTime && Revol_Switch_Left == 2 //�뵯�ֿ���ͬ��
		&& Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
	// Fric_mode(FRI_HIGH);
}

/**
  * @brief  ��������
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_TRIPLE_Ctrl(void)
{

	Revolver_mode = REVOL_SPEED_MODE;
	shooter_heat_cooling = JUDGE_usGetShootCold();
	if (JUDGE_usGetShootCold() <= 40)
	{
		Revolver_Freq = 8; //��Ƶѡ��
		Fric_mode(FRI_LOW);
	}
	else if (JUDGE_usGetShootCold() <= 60 && JUDGE_usGetShootCold() > 40)
	{
		Revolver_Freq = 8; //10;//��Ƶѡ��
		Fric_mode(FRI_MID);
	}
	else if (JUDGE_usGetShootCold() <= 80 && JUDGE_usGetShootCold() > 60)
	{
		Revolver_Freq = 8; //12;//��Ƶѡ��
		Fric_mode(FRI_MID);
	}
	else if (JUDGE_usGetShootCold() >= 160) //ռ��ﱤ
	{
		Revolver_Freq = 14; //12;//��Ƶѡ��
		Fric_mode(FRI_HIGH);
	}
	else
	{
		Revolver_Freq = 8; //��Ƶѡ��
		Fric_mode(FRI_LOW);
	}

	//�ٶȻ�ת������
	Revolver_Speed_Target = -REVOL_SPEED_RATIO / REVOL_SPEED_GRID * Revolver_Freq;
}

/**
  * @brief  ����Ƶ�����ٿ���
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_HIGHTF_LOWS_Ctrl(void)
{
	static portTickType CurrentTime = 0;
	static uint32_t RespondTime = 0; //��Ӧ�����ʱ

	CurrentTime = xTaskGetTickCount(); //��ǰϵͳʱ��

	Shoot_Interval = TIME_STAMP_1000MS / 25; //ȷ����Ƶ

	if (RespondTime < CurrentTime && Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
	Fric_mode(FRI_LOW);
}

/**
  * @brief  ����Ƶ�����ٿ���
  * @param  void
  * @retval void
  * @attention  
  */
void SHOOT_MIDF_HIGHTS_Ctrl(void)
{
	static portTickType CurrentTime = 0;
	static uint32_t RespondTime = 0; //��Ӧ�����ʱ

	CurrentTime = xTaskGetTickCount(); //��ǰϵͳʱ��

	Shoot_Interval = TIME_STAMP_1000MS / 20; //ȷ����Ƶ

	if (RespondTime < CurrentTime && Key_ShootNum == 0)
	{
		RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum++;
	}
	Fric_mode(FRI_LOW);
}

/**
  * @brief  ���������ƣ�����ͷ����̨
  * @param  void
  * @retval void
  * @attention  ÿ��500ms�ж�һ�ε�λ����λ����
  */
uint32_t buff_lost_time = 0;		//��֡��ʱ������ʱ�����Ϊ��֡
uint32_t buff_change_lost_time = 0; //��װ�׵�֡��ʱ
uint32_t buff_shoot_close = 0;
float buff_stamp = 800;		 //1100;//��0.8�벹һ��
float buff_lost_stamp = 200; //����200ms��ʧĿ��
float Armor_Change_Delay = 0;
float dy = 80; //1;//130;
void SHOOT_BUFF_Ctrl_Gimbal(void)
{
	portTickType CurrentTime = 0;
	static uint32_t RespondTime = 0; //��Ӧ�����ʱ
	static uint32_t lockon_time = 0; //�ȶ���׼һ��ʱ���ɻ���

	CurrentTime = xTaskGetTickCount();

	if (!IF_MOUSE_PRESSED_RIGH) //�Ҽ��ɿ��Զ���
	{
		//��֡ͳ�ƣ���̫֡���ز���
		if (VisionRecvData.identify_buff == FALSE) //ûʶ��Ŀ��
		{
			buff_lost_time++;
			if (buff_lost_time > buff_lost_stamp)
			{
				buff_fire = FALSE;
			}
		}
		else
		{
			buff_lost_time = 0;
			buff_fire = TRUE; //���Կ���
		}

		Shoot_Interval = 200; //���������Ƶ,����̫���ֹ����

		//ʶ��Ŀ���ҽӽ�Ŀ��
		if (buff_fire == TRUE			   //�ǳ�ʱ���֡
			&& Vision_If_Armor() == FALSE) //��װ���л�
		{
			//			buff_shoot_close = 0;//���¼���δ��׼Ŀ��ʱ��
			Armor_Change_Delay++;
			if (Armor_Change_Delay > 50)
			{
				if (GIMBAL_BUFF_YAW_READY() && GIMBAL_BUFF_PITCH_READY())
				{
					buff_change_lost_time = 0; //ˢ�µ�λ�ж�ʱ��
					buff_change_fire = TRUE;   //�������л�����ȶ���ʱ
				}
				else
				{
					buff_change_lost_time++;		//�ȶ���λ
					if (buff_change_lost_time > 50) //������֡50ms����Ϊû��λ
					{
						buff_change_fire = FALSE; //�����л��ȶ���ʱ
					}
				}

				if (buff_change_fire == TRUE)
				{
					lockon_time++;
				}
				else
				{
					lockon_time = 0;
				}

				if (RespondTime < CurrentTime && Key_ShootNum == 0 && lockon_time > dy	   //80
					&& (VisionRecvData.yaw_angle != 0 && VisionRecvData.pitch_angle != 0)) //�ȶ���λ30ms
				{
					RespondTime = CurrentTime + buff_stamp; //Shoot_Interval;
					Key_ShootNum++;							//��һ��
				}
			}
		}
		else //��ʱ���֡�����л���װ��
		{
			lockon_time = 0;
			buff_change_fire = FALSE; //�����л��ȶ���ʱ

			if (Vision_If_Armor() == TRUE) //�л�װ�װ�
			{
				Armor_Change_Delay = 0;
				Vision_Clean_Ammor_Flag(); //�ȴ��´θ���װ�װ�
			}

			RespondTime = CurrentTime - 1; //����ˢ�����ʱ��
			Key_ShootNum = 0;

			//			buff_shoot_close++;
			//			lockon_time = 0;
			//			if(buff_shoot_close > 100)//����100msû�鵽Ŀ��
			//			{
			//				RespondTime = CurrentTime-1;//����ˢ�����ʱ��
			//				Key_ShootNum = 0;
			//			}
		}
	}
	else //��ס�Ҽ��ӹ��Զ���
	{
		/*------ ���̧�����ܴ���һ�� -------*/
		if (IF_MOUSE_PRESSED_LEFT)
		{
			if (Revol_Switch_Left == 1)
			{
				Revol_Switch_Left = 2;
			}
			else if (Revol_Switch_Left == 2)
			{
				Revol_Switch_Left = 0;
			}
		}
		else if (!IF_MOUSE_PRESSED_LEFT)
		{
			Revol_Switch_Left = 1;
		}

		/*------------------------------------*/
		if (IF_MOUSE_PRESSED_LEFT) //�������
		{
			SHOOT_SINGLE_Ctrl();
		}
	}
}

/**
  * @brief  ����ģʽ�����ٶȻ�����
  * @param  void
  * @retval void
  * @attention �Ƽ�ģʽ������Ƶ,�����ǵü���һ����־λ��������Ħ���ֵ�������
  */
void REVOLVER_KeySpeedCtrl(void)
{
	REVOL_SpeedStuck(); //�����жϼ���ת
}

/**
  * @brief  ����ģʽ����λ�û�����
  * @param  void
  * @retval void
  * @attention 
  */
void REVOLVER_KeyPosiCtrl(void)
{
	static portTickType CurrentTime = 0;
	//	static uint32_t  RespondTime = 0;//��Ӧ�����ʱ

	CurrentTime = xTaskGetTickCount();

	if (Key_ShootNum != 0 && Revol_Posit_RespondTime < CurrentTime)
	{
		Revol_Posit_RespondTime = CurrentTime + Shoot_Interval;
		Key_ShootNum--;						   //����������
		Revolver_Buff_Target_Sum -= AN_BULLET; //����λ�ü�

		posishoot_time = xTaskGetTickCount(); //����ָ���´�ʱ��ϵͳʱ��,���ڷ�����ʱ����
	}

	if (Revolver_Angle_Target_Sum != Revolver_Buff_Target_Sum) //����ת��ȥ
	{
		Revolver_Angle_Target_Sum = RAMP_float(Revolver_Buff_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Buff_Ramp);
	}

	REVOL_PositStuck(); //�����жϼ���ת,��ǰ�������һ��
}

/**
  * @brief  ����ң�ش�
  * @param  void
  * @retval void
  * @attention 
  */
#define REVOL_STEP0 0		 //ʧ�ܱ�־
#define REVOL_STEP1 1		 //SW1��λ��־
#define REVOL_STEP2 2		 //���ֿ��ر�־
uint8_t Revolver_Switch = 0; //����ң��ģʽ���ر�־λת��
bool REVOLVER_Rc_Switch(void)
{
	if (IF_RC_SW2_MID) //��еģʽ
	{
		if (IF_RC_SW1_DOWN)
		{
			if (Revolver_Switch == REVOL_STEP1)
			{
				Revolver_Switch = REVOL_STEP2;
			}
			else if (Revolver_Switch == REVOL_STEP2)
			{
				Revolver_Switch = REVOL_STEP0;
			}
		}
		else
		{
			Revolver_Switch = REVOL_STEP1;
		}
	}
	else
	{
		Revolver_Switch = REVOL_STEP0;
	}

	if (Revolver_Switch == REVOL_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �ٶȻ�ʽ��������
  * @param  void
  * @retval void
  * @attention  ��ס�ͷ�תn��
  */
void REVOL_SpeedStuck(void)
{
	static uint16_t stuck_time = 0;			 //������ʱ
	static uint16_t turnback_time = 0;		 //��ת��ʱ
	static bool Revol_Speed_ifStuck = FALSE; //�����ж�

	if (Revol_Speed_ifStuck == TRUE) //��ȷ�Ͽ���,��ʼ��ת��ʱ
	{
		Revolver_Speed_Target = -4000; //��ת
		turnback_time++;			   //��תһ��ʱ��

		if (turnback_time > Stuck_TurnBack_Time) //��ת���
		{
			turnback_time = 0;
			Revol_Speed_ifStuck = FALSE; //������ת
		}
	}
	else
	{
		if (fabs(Revolver_Final_Output) >= Stuck_Revol_PIDTerm //PID�������
			&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low) //�ٶȹ���
		{
			stuck_time++; //������ʱ
		}
		else
		{
			stuck_time = 0; //û�г�ʱ�俨��,��ʱ����
		}

		if (stuck_time > Stuck_SpeedPID_Time) //���˳���60ms
		{
			Stuck_Speed_Sum++; //��������,����е��Ĵ��ֵܵ�����
			stuck_time = 0;
			Revol_Speed_ifStuck = TRUE; //��ǿ��Խ��뵹ת��ʱ
		}
	}
}

/**
  * @brief  λ�û�ʽ��������
  * @param  void
  * @retval void
  * @attention  ��ס�ͷ�תn��
  */
void REVOL_PositStuck(void)
{
	static uint16_t stuck_time = 0;			 //������ʱ
	static uint16_t turnback_time = 0;		 //��ת��ʱ
	static bool Revol_Posit_ifStuck = FALSE; //�����ж�

	if (Revol_Posit_ifStuck == TRUE) //������ʼ��ת��ʱ
	{
		//�����������ж��Ƿ񿨵����ʱ������갴�µ�ָ�����
		Key_ShootNum = 0;

		turnback_time++;						 //��ת��ʱ,1msһ��0/
		if (turnback_time > Stuck_TurnBack_Time) //��תʱ�乻��
		{
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum; //������ת,��ת�ر�����Ҫ��ת����λ��
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;
			Revol_Posit_ifStuck = FALSE; //��Ϊ��ʱ���ٿ�����
			turnback_time = 0;			 //��תʱ������,Ϊ�´ε�ת��׼��
		}
	}
	else
	{
		if (fabs(Revolver_Final_Output) >= Stuck_Revol_PIDTerm //PID�������
			&& abs(Revolver_Speed_Measure) <= Stuck_Speed_Low) //�ٶȹ���
		{
			stuck_time++; //ͳ�ƿ��˶೤ʱ��
		}
		else
		{
			stuck_time = 0; //������,ʱ������
		}

		if (stuck_time > Stuck_SpeedPID_Time) //��̫����,��ʾҪ��ת
		{
			//��ת���ܷ���Revol_Posit_ifStuck == TRUE��,����Ͳ��Ƕ�һ�ε�ת1/2����
			Revolver_Angle_Target_Sum = Revolver_Angle_Measure_Sum + AN_BULLET; //��ת 1��
			Revolver_Buff_Target_Sum = Revolver_Angle_Target_Sum;

			Stuck_Posit_Sum++; //��������,����е��Ĵ��ֵܵ�����
			stuck_time = 0;
			Revol_Posit_ifStuck = TRUE; //������ǵ�ת��ʱ����
		}
	}
}

/***********************PID����**********************/
/**
  * @brief  �ٶȻ�PID����
  * @param  void
  * @retval void
  * @attention  ң��ֻ���ٶȻ�
  */
void REVOL_SpeedLoop(void)
{
	Revolver_Speed_Error = Revolver_Speed_Target - Revolver_Speed_Measure;

	//���͵���PID�㷨
	pTermRevolSpeed = Revolver_Speed_Error * Revolver_Speed_kpid[KP];
	iTermRevolSpeed += Revolver_Speed_Error * Revolver_Speed_kpid[KI];
	iTermRevolSpeed = constrain(iTermRevolSpeed, -iTermRevolSpeedMax, iTermRevolSpeedMax);

	Revolver_Final_Output = constrain_float(pTermRevolSpeed + iTermRevolSpeed, -Revolver_Output_Max, +Revolver_Output_Max);
}

/**
  * @brief  λ�û�PID����
  * @param  void
  * @retval void
  * @attention  ����ģʽ
  */
void REVOL_PositionLoop(void)
{
	//��ȡת�����ܽǶ�ֵ
	REVOL_UpdateMotorAngleSum();

	//�⻷����
	Revolver_Angle_Error[OUTER] = Revolver_Angle_Target_Sum - Revolver_Angle_Measure_Sum;
	pTermRevolAngle[OUTER] = Revolver_Angle_Error[OUTER] * Revolver_Angle_kpid[OUTER][KP];

	//�ڻ�����
	Revolver_Angle_Error[INNER] = pTermRevolAngle[OUTER] - Revolver_Speed_Measure;
	pTermRevolAngle[INNER] = Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KP];
	iTermRevolAngle[INNER] += Revolver_Angle_Error[INNER] * Revolver_Angle_kpid[INNER][KI] * 0.001f;
	iTermRevolAngle[INNER] = constrain_float(iTermRevolAngle[INNER], -iTermRevolPosiMax, iTermRevolPosiMax);

	Revolver_Final_Output = constrain_float(pTermRevolAngle[INNER] + iTermRevolAngle[INNER], -Revolver_Output_Max, Revolver_Output_Max);
}

/**
  * @brief  ͳ��ת���Ƕ��ܺ�
  * @param  void
  * @retval void
  * @attention �л���ģʽ֮��ǵ����� 
  */
void REVOL_UpdateMotorAngleSum(void)
{
	//�ٽ�ֵ�жϷ�
	if (abs(Revolver_Angle_Measure - Revolver_Angle_Measure_Prev) > 4095) //ת����Ȧ
	{
		//���β����Ƕ�С���ϴβ����Ƕ��ҹ��˰�Ȧ,��˵�����ι������
		if (Revolver_Angle_Measure < Revolver_Angle_Measure_Prev) //����Ȧ�ҹ����
		{
			//�Ѿ�ת����һȦ,���ۼ�ת�� 8191(һȦ) - �ϴ� + ����
			Revolver_Angle_Measure_Sum += 8191 - Revolver_Angle_Measure_Prev + Revolver_Angle_Measure;
		}
		else
		{
			//��ʱ��ת
			Revolver_Angle_Measure_Sum -= 8191 - Revolver_Angle_Measure + Revolver_Angle_Measure_Prev;
		}
	}
	else
	{
		//δ���ٽ�ֵ,�ۼ���ת���ĽǶȲ�
		Revolver_Angle_Measure_Sum += Revolver_Angle_Measure - Revolver_Angle_Measure_Prev;
	}

	//��¼��ʱ����Ƕ�,��һ�μ���ת���ǶȲ���,�����ж��Ƿ�ת��1Ȧ
	Revolver_Angle_Measure_Prev = Revolver_Angle_Measure;
}

/*******************���̵�����ݸ���*********************/

/**
  * @brief  ��ȡ����Ƕ�
  * @param  CAN����
  * @retval void
  * @attention  CAN1�ж��е���
  */
void REVOLVER_UpdateMotorAngle(int16_t angle)
{
	Revolver_Angle_Measure = angle;
}

/**
  * @brief  ��ȡ���ת��
  * @param  CAN����
  * @retval void
  * @attention  CAN1�ж��е���
  */
void REVOLVER_UpdateMotorSpeed(int16_t speed)
{
	Revolver_Speed_Measure = speed;
}

/**
  * @brief  ���Ͳ��̵���ֵ
  * @param  void
  * @retval void
  * @attention 
  */
void SHOOT_CANbusCtrlMotor(void)
{
	CAN_CMD_SHOOT(Fric_Final_Output[Fric_Left], Fric_Final_Output[Fric_Right], Revolver_Final_Output, 0); //Revolver_Final_Output
}

/**
  * @brief  Ħ����ң�ؿ���
  * @param  void
  * @retval �Ƿ�ת����ǰ״̬
  * @attention �����ֿ����߼���ͬ
  */
bool FRIC_RcSwitch(void)
{
	if (IF_RC_SW1_DOWN) //����Ħ��������1
	{
		if (Friction_Switch == FRIC_STEP1)
		{
			Friction_Switch = FRIC_STEP2;
		}
		else if (Friction_Switch == FRIC_STEP2)
		{
			Friction_Switch = FRIC_STEP0; //�ж���ϵ
		}
	}
	else //��־SW1�Ƿ��и�λ�����,�ڸ�λ������²����ٴν���STERP2
	{
		Friction_Switch = FRIC_STEP1; //����SW1���´α任֮ǰһֱ������
	}

	if (Friction_Switch == FRIC_STEP2)
	{
		return TRUE; //ֻ��SW1���±任��ʱ���ΪTRUE
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  ����Ħ�������ٸ��ٵ���
  * @param  void
  * @retval void
  * @attention 
  */
void Fric_mode(uint16_t speed)
{
	if (FRIC_RcSwitch() == TRUE) //�ж�״̬�л�
	{
		if (speed == FRI_LOW)
		{
			Friction_Speed_Target = Friction_PWM_Output[FRI_LOW];
		}

		else if (speed == FRI_SENTRY)
		{
			Friction_Speed_Target = Friction_PWM_Output[FRI_SENTRY];
		}

		else if (speed == FRI_HIGH)
		{
			Friction_Speed_Target = Friction_PWM_Output[FRI_HIGH];
		}

		else if (speed == FRI_MAD)
		{
			Friction_Speed_Target = Friction_PWM_Output[FRI_HIGH];
		}

		else if (speed == FRI_MID)
		{
			Friction_Speed_Target = Friction_PWM_Output[FRI_MID];
		}
	}
	if (speed == FRI_OFF)
	{
		Friction_Speed_Target = Friction_PWM_Output[FRI_OFF];
	}

	Friction_Ramp();

	Fric_Open(Friction_Speed_Real, Friction_Speed_Real);
}

/**
  * @brief  Ħ�������б��
  * @param  void
  * @retval void
  * @attention 
  */
void Friction_Ramp(void)
{
	if (Friction_Speed_Real < Friction_Speed_Target) //����
	{
		Friction_Speed_Real += 2;
		if (Friction_Speed_Real > Friction_Speed_Target)
		{
			Friction_Speed_Real = Friction_Speed_Target;
		}
	}
	else if (Friction_Speed_Real > Friction_Speed_Target) //�ر�
	{
		Friction_Speed_Real -= 2;
	}

	if (Friction_Speed_Real < 0)
	{
		Friction_Speed_Real = 0;
	}
}

/**
  * @brief  stop Fric
  * @param  void
  * @retval void
  * @attention 
  */
void Reset_Fric(void)
{
	Fric_Speed_Level = FRI_OFF;
	Friction_Speed_Target = 0;
	Friction_Speed_Real = 0;
}

//ģ�����ƽ̨�õ�
//#define    CORGI_BEGIN    0
//#define    CORGI_LEFT     1
//#define    CORGI_RIGH     2
//float  Revolver_Shake_Target_Sum;  //ģ�����ת��
//float Revolver_angle_Target;    //�������
//float Revolver_Shake_Ramp = AN_BULLET/80;   //500ms
//extern uint16_t  stateCorgi = CORGI_BEGIN;//�������Ť,Ĭ�ϲ�Ť
//void REVOLVER_SHAKE(void)
//{

//
//	switch(stateCorgi)
//	{
//		case CORGI_BEGIN:
//
//			Revolver_Shake_Target_Sum = measure_first;
//		  Revolver_Shake_Target_Sum =measure_first+AN_BULLET ;
//		  if(Revolver_Angle_Target_Sum!=Revolver_Shake_Target_Sum)
//			{
//				Revolver_Angle_Target_Sum=RAMP_float(Revolver_Shake_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Shake_Ramp);
//			}
//			if(Revolver_Angle_Measure_Sum > (Revolver_Shake_Target_Sum-200))
//			{
//		    stateCorgi = CORGI_RIGH;
//			}
//		break;
//
//		case CORGI_LEFT:
//			Revolver_Shake_Target_Sum = measure_first+AN_BULLET;
//		  Revolver_Shake_Target_Sum =measure_first-AN_BULLET;
//		  if(Revolver_Angle_Target_Sum!=Revolver_Shake_Target_Sum)
//			{
//				Revolver_Angle_Target_Sum=RAMP_float(Revolver_Shake_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Shake_Ramp);
//			}
//			if(Revolver_Angle_Measure_Sum < (Revolver_Shake_Target_Sum+200))
//			{
//		    stateCorgi = CORGI_RIGH;
//			}
//		break;
//
//		case CORGI_RIGH:
//
//			Revolver_Shake_Target_Sum = measure_first+AN_BULLET;
//		  Revolver_Shake_Target_Sum =measure_first+AN_BULLET*3;
//		  if(Revolver_Angle_Target_Sum!=Revolver_Shake_Target_Sum)
//			{
//				Revolver_Angle_Target_Sum=RAMP_float(Revolver_Shake_Target_Sum, Revolver_Angle_Target_Sum, Revolver_Shake_Ramp);
//			}
//			if(Revolver_Angle_Measure_Sum > (Revolver_Shake_Target_Sum-200))
//			{
//		    stateCorgi = CORGI_LEFT;
//			}
//		break;
//

//	}
//
//}
