/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
  * @note        ������дһ��shif���ٹ��� ��Ҫ��ϳ�������
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
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "Gimbal_Task.h"
#include "start_task.h"
#include "rc.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "led.h"
#include "super_cap.h"
#include "arm_math.h"

#include "CAN_Receive.h"

#include "pid.h"
#include "stdio.h"
#include <stdlib.h>
#include "Remote_Control.h"
#include "usart6.h"

#include "kalman.h"
#include "judge.h"
#include "magazine.h"

#define rc_deadline_limit(input, output, dealine)        \
	{                                                    \
		if ((input) > (dealine) || (input) < -(dealine)) \
		{                                                \
			(output) = (input);                          \
		}                                                \
		else                                             \
		{                                                \
			(output) = 0;                                \
		}                                                \
	}

extern RC_ctrl_t rc_ctrl;
ChassisCtrlMode Chassis_Mode;	  //��еģʽ ������ģʽ
ChassisActionMode Chassis_Action; //���̶�����̬
extKalman_t Chassis_Error_Kalman; //����һ��kalmanָ��

PidTypeDef motor_pid[4];
PidTypeDef motor_key_pid[4];
PidTypeDef chassis_angle_pid;
PidTypeDef chassis_angle_key_pid;

first_order_filter_type_t chassis_cmd_slow_set_vx;
first_order_filter_type_t chassis_cmd_slow_set_vy;

//С����
bool Chass_Switch_F = 1;
u8 Chass_Key_F_Change = 0;

//�Զ�����
#define MISS_MAX_TIME 1000	 //�Զ���������λʱ��,��λ2*ms
uint32_t Miss_Mode_Time = 0; //�Զ������ѹ�ʱ��

//������
uint8_t remote_change = TRUE;

//���̹��ʸ��ı���
uint16_t Chassis_Mode = 0;

void chassis_task(void *pvParameters)
{
	//    //����һ��ʱ��
	vTaskDelay(CHASSIS_TASK_INIT_TIME); //ʱ��Ϊ357

	for (;;)
	{
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING) //ϵͳ״̬Ϊ��ʼ
		{
			Chassis_Init();
			Cap_Init();
		}
		else
		{
			if (IF_RC_SW2_UP) //����ģʽ
			{
				chassis_feedback_update();
				Chassis_Mode_Set();        //���̹��ʻ���
				Chassis_Key_Ctrl();		   //���̰�������
				Chassis_Set_key_Contorl(); //�����ƶ�����
			}
			else //ң����ģʽ
			{
				Chassis_Set_Mode();		   //�л�ģʽ
				chassis_feedback_update(); //��������
				Chassis_Rc_Control();	   //ң��������
				Chassis_Set_Contorl();	   //��ͬģʽ��ͬ����

				/*�л�����ģʽʱ��ı�����ʼ��*/
				Chassis_Action = CHASSIS_NORMAL;
				Chass_Switch_F = 1;		//����Ť��
				Chass_Key_F_Change = 0; //����Ť��
				remote_change = TRUE;	//����л���ң��ģʽ
			}
		}
		Chassis_Omni_Move_Calculate(); //����ȫ���˶�����
		if (IF_RC_SW2_UP)
		{
			Chassis_Motor_Speed_PID_KEY(); //PID����
		}
		else
		{
			Chassis_Motor_Speed_PID(); //PID����
		}

		CHASSIS_CANSend(); //���͵���
		vTaskDelay(TIME_STAMP_2MS);
	}
}

/*----------------------myself-----------------------*/

/*----------------------------------------------***�������ȫ���ƶ�����***-------------------------------------------------*/

float Chassis_Move_X; //ǰ��
float Chassis_Move_Y; //����ƽ��
float Chassis_Move_Z; //������ת

fp32 angle_swing_set; //ҡ�ڽǶ�

//������ģʽ�µ���ƫ��(���YAW�����ķ����е�Ƕ�)
float Chassis_Gyro_Error;

fp32 motor_chassis_speed[4];	 //�����ĸ����ӵ��ٶȣ����ڸ���   m/s
fp32 motor_key_chassis_speed[4]; //�����ĸ����ӵ��ٶȣ����ڸ���   m/s
/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***�ٶ��޷�***-------------------------------------------------------------*/

fp32 vx_max_speed; //ǰ����������ٶ� ��λm/s
fp32 vx_min_speed; //ǰ��������С�ٶ� ��λm/s
fp32 vy_max_speed; //���ҷ�������ٶ� ��λm/s
fp32 vy_min_speed; //���ҷ�����С�ٶ� ��λm/s
fp32 vz_max_speed; //���ҷ�������ٶ� ��λm/s
fp32 vz_min_speed; //���ҷ�����С�ٶ� ��λm/s

float theta;
float theta_error1;
float theta_error2;

//����ģʽ �޷�
float Chassis_Standard_Move_Max;					//����ǰ������ƽ������
float Chassis_Revolve_Move_Max;						//����������ת����,���ݲ�ͬ�˶�ģʽʵʱ����,���Բ����ú궨��
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX; //����PID�����������ֵ,���̹�������,���ݵ��ת��ʵʱ�仯
//����ģʽ ��Ӧ  ��ʼ����Ҫ����
float kKey_Mech_Chassis_Standard; //�����еģʽƽ�Ƽ�����Ӧ
float kKey_Mech_Chassis_Revolve;  //�����еģʽ��ת������Ӧ
float kKey_Gyro_Chassis_Standard; //����������ģʽƽ�Ƽ�����Ӧ
float kKey_Gyro_Chassis_Revolve;  //����������ģʽ��ת������Ӧ
/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***б�²���***-------------------------------------------------------------*/
float Slope_Chassis_Move_Z; //б�¼�������ƶ�����,����Ŀ���������ʵʱб��ֵ

uint16_t timeInc;									//б�����ӱ仯ʱ��
uint16_t timeInc_Saltation;							//ǰ����ͻ���µ�б��������,�����������ҪС
int16_t timeXFron, timeXBack, timeYLeft, timeYRigh; //����  s  w  d   a

//����ģʽ��ȫ���ƶ�����,б����
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back;
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//����ģʽ��Ťͷб��,��Ҫ����Ťƨ��ģʽ��
float Slope_Chassis_Revolve_Move;

/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***����PID����***--------------------------------------------------------------*/

//������̨�Ƕ���������̵�ŷ����
extern float Cloud_Angle_Measure[2][3]; //  pitch/yaw    mech/gyro

//���������ٶ�
float Chassis_Speed_Target[4]; //ID

//�����ٶ����
float Chassis_Speed_Error[4]; //ID

//���̲����Ƕ�
int16_t Chassis_Angle_Measure[4];

//���̲����ٶ�
float Chassis_Speed_Measure[4];

//���̲����ٶ�
int16_t Chassis_Current_Measure[4];

//��һ��pid

//���̲����ٶ�
int16_t Chassis_Current_Measure[4];

//�����ٶ�����
float Chassis_Speed_Error_Sum[4]; //ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];

//����PID����
float Chassis_Speed_kpid[4][3]; //	motorID kp/ki/kd

float pTermChassis[4], iTermChassis[4], dTermChassis[4]; //ID
float pidTermChassis[4];								 //ID,���������

/*----------------------------------------------------------------------------------------------------------------------*/

//���̵�������
float Chassis_Final_Output[4];

/**
  * @brief  ��ʼ���������
  * @param  void
  * @retval void
  * @attention  Chassis_Move_X Chassis_Move_Y Chassis_Move_Z ��0
  */
void CHASSIS_REST(void)
{
	Slope_Chassis_Move_Z = 0; //Ťƨ��ʵʱ���б��
	Chassis_Move_X = 0;
	Chassis_Move_Y = 0;
	Chassis_Move_Z = 0;
}

/**
  * @brief  ���̳�ʼ��  ��Ҫ��PID �޷���ʼ��
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Init(void)
{
	//�����ٶȻ�pidֵ
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};				   //��ʼ��ң����ģʽKP��KI��KD
	const static fp32 motor_speed_key_pid[3] = {M3505_MOTOR_SPEED_KEY_PID_KP, M3505_MOTOR_SPEED_KEY_PID_KI, M3505_MOTOR_SPEED_KEY_PID_KD}; //��ʼ������ģʽKP��KI��KD
	//������ת��pidֵ
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};				   //��ʼ��������ת����KP��KI��KD
	const static fp32 chassis_yaw_key_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KEY_KP, CHASSIS_FOLLOW_GIMBAL_PID_KEY_KI, CHASSIS_FOLLOW_GIMBAL_PID_KEY_KD}; //��ʼ��������ת����KP��KI��KD �ļ���ģʽPID

	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM}; //����x����һ�׵�ͨ�˲�����
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM}; //����y����һ�׵�ͨ�˲�����
	uint8_t i;															 //ѭ������,ֻ�ں���ѭ����

	Chassis_Mode = CHASSIS_MECH_MODE; //��ʼ������ģʽΪ��еģʽ

	//��ʼ��PID �˶�
	for (i = 0; i < 4; i++)
	{

		PID_Init(&motor_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT); //�ڶ�����������������λ�û�
		PID_Init(&motor_key_pid[i], PID_POSITION, motor_speed_key_pid, M3505_MOTOR_SPEED_KEY_PID_MAX_OUT, M3505_MOTOR_SPEED_KEY_PID_MAX_IOUT);
	}

	//��ʼ����תPID
	PID_Init(&chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	PID_Init(&chassis_angle_key_pid, PID_POSITION, chassis_yaw_key_pid, CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_IOUT);
	//��һ���˲�����б����������
	first_order_filter_init(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;	//����ǰ��������ٶ�
	vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X; //���̺��˵�����ٶ�

	vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;	//������ƽ�Ƶ�����ٶ�   //����debug��
	vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y; //������ƽ�Ƶ�����ٶ�

	vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;	//����˳ʱ����ת������ٶ�
	vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z; //������ʱ����ת������ٶ�

	/**************����ģʽ�޷�����*************************/
	Chassis_Standard_Move_Max = STANDARD_MAX_NORMAL; //����ˮƽ�ƶ��޷�  9000
	Chassis_Revolve_Move_Max = REVOLVE_MAX_NORMAL;	 //��������Ťͷ�޷�,������΢��һ��,������̨Ť̫��ᵼ����̨ײ����λ,̫���ֻᵼ�µ�Ŀ��λ�ú�ζ� 9000

	/**************���̰���������*************************/
	kKey_Mech_Chassis_Revolve = 60;	 //���̻�еģʽ��Ťͷ�ٶ���Ӧ����,��̫��,��ȻŤͷ̫��
	kKey_Gyro_Chassis_Revolve = -10; //-8.1;//ע������,����������ģʽ�µ��̸�����̨ת�����ٶ�,���̫��,�����𵴻������

	/**************PID����*************************/
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 18;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;

	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 18;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;

	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 18;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;

	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 18;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;

	chassis_feedback_update();
}

/**
  * @brief  ���µ��̵������   �ٶ�  ����̨�ĵ��ӽǶ�
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void chassis_feedback_update(void)
{
	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		motor_chassis_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Chassis_Speed_Measure[i];	 //ת����m/s��ת��
		motor_key_chassis_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Chassis_Speed_Measure[i]; //ת����m/s��ת��
	}
	Chassis_Gyro_Error = GIMBAL_GetOffsetAngle();
}

/**
  * @brief  ң�����л�ģʽ�����޸���Ӧ����
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Mode(void)
{
	if (IF_RC_SW2_MID)
	{
		Chassis_Mode = CHASSIS_GYRO_MODE; //S2�� ������ģʽ
		flow_led_on(3);
		flow_led_off(2);
	}
	else if (IF_RC_SW2_UP)
	{
		Chassis_Mode = CHASSIS_MECH_MODE; //������еģʽ
		angle_swing_set = 0;
		flow_led_on(2);
		flow_led_off(3);
	}
	else if (IF_RC_SW2_DOWN)
	{
		Reset_Fric();
		Chassis_Mode = CHASSIS_TOP_MODE; //������еģʽ
		angle_swing_set = 0;
		flow_led_on(2);
		flow_led_off(3);
	}
	//	if(IF_RC_SW1_DOWN)
	//	{
	//		Chassis_Mode = CHASSIS_SHAKE_MODE;
	//		flow_led_on(2);
	//		flow_led_on(3);
	//	}
}

/**
  * @brief  ң������������Ϊ Chassis_Move_X  Chassis_Move_Y
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Rc_Control(void)
{
	//ң����ԭʼͨ��ֵ
	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	//�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0               ��ң��������ֵ��Сʱ���Ϊ0����ֹ��
	rc_deadline_limit(rc_ctrl.rc.ch[3], vy_channel, CHASSIS_RC_DEADLINE);
	rc_deadline_limit(rc_ctrl.rc.ch[2], vx_channel, CHASSIS_RC_DEADLINE);

	vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN; //��ң������ֵת��Ϊ�������˶����ٶ�
	vy_set_channel = vy_channel * CHASSIS_VY_RC_SEN;

	//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
	first_order_filter_cali(&chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_cmd_slow_set_vy, vy_set_channel);

	//ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_cmd_slow_set_vy.out = 0.0f;
	}
	Chassis_Move_X = chassis_cmd_slow_set_vx.out; //ң�������X�᷽���ֵ
	Chassis_Move_Y = chassis_cmd_slow_set_vy.out; //ң�������Y�᷽���ֵ
}

/**
  * @brief  ��ͬģʽ��ͬ����ʽ
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Contorl(void)
{
	float Chassis_Move_X1;
	float Chassis_Move_Y1;
	if (Chassis_Mode == CHASSIS_SHAKE_MODE) //Ť��ģʽ
	{
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

		float C_A_M_Y_M = 0;

		chassis_follow_gimbal_yaw_control(); //�Ƿ���Ҫҡ��      ������õ�angle_swing_set

		//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
		sin_yaw = arm_sin_f32(Cloud_Angle_Measure[YAW][MECH]);
		cos_yaw = arm_cos_f32(Cloud_Angle_Measure[YAW][MECH]);
#if YAW_POSITION == YAW_DOWN //�������¶�ȡ�ĵ���Ƕ�ֵ�Ƿ���
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][CHANGE];
#else
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][MECH];
#endif

		Chassis_Move_X = cos_yaw * Chassis_Move_X + sin_yaw * Chassis_Move_Y; //�����ǿ����ƶ�ʱ��Ҫ�õ�
		Chassis_Move_Y = -sin_yaw * Chassis_Move_X + cos_yaw * Chassis_Move_Y;

		//������תPID���ٶ�
		Chassis_Move_Z = PID_Calc(&chassis_angle_pid, C_A_M_Y_M, rad_format(angle_swing_set));
		//�ٶ��޷�
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}

	else if (Chassis_Mode == CHASSIS_GYRO_MODE)
	{
		Angle_error();
		//������ת�Ľ��ٶ�
		Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f, Chassis_Gyro_Error); //���赱ǰʵ��ֵΪ0���趨���ֵΪĿ��ֵ������趨��ǰĿ��ֵΪ0��ʵ��ֵΪ���ֵ����ʱ���ֵӦȡ�������˶�����ȡ����
		//���õ����˶����ٶ�
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_MECH_MODE)
	{
		Angle_error();
		Chassis_Move_Z = CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[0];
		Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_TOP_MODE)
	{
		if ((fabs(Chassis_Move_X) < 0.001) && (fabs(Chassis_Move_Y) < 0.001))
		{
			Chassis_Move_Z = 1.5f;
		}
		else
		{
			Chassis_Move_Z = 1.0f;
			Angle_error();

			Chassis_Move_X1 = (cos(theta) * Chassis_Move_X) + (-sin(theta) * Chassis_Move_Y);
			Chassis_Move_Y1 = (sin(theta) * Chassis_Move_X) + (cos(theta) * Chassis_Move_Y);
			Chassis_Move_X = fp32_constrain(Chassis_Move_X1, vx_min_speed, vx_max_speed);
			Chassis_Move_Y = fp32_constrain(Chassis_Move_Y1, vy_min_speed, vy_max_speed);
			Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
		}
	}
}

/**
  * @brief  ����
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Mode_Set(void)
{
	if(IF_KEY_PRESSED_CTRL || IF_KEY_PRESSED_SHIFT)
	{ 
		if (IF_KEY_PRESSED_CTRL)
		{
			Chassis_Mode ++;
			if (Chassis_Mode > 2)
			{
				Chassis_Mode = 2;
			}
		}
		else if (IF_KEY_PRESSED_SHIFT)
		{
			Chassis_Mode --;
			if (Chassis_Mode < 0)
			{
				Chassis_Mode = 0;
			}
		}
}


/**
  * @brief  ��ͬģʽ��ͬ����ʽ
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_key_Contorl(void)
{
	float Chassis_Move_X1;
	float Chassis_Move_Y1;
	if (Chassis_Mode == CHASSIS_SHAKE_MODE) //Ť��ģʽ
	{
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

		float C_A_M_Y_M = 0;

		chassis_follow_gimbal_yaw_control(); //�Ƿ���Ҫҡ��      ������õ�angle_swing_set

		//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
		sin_yaw = arm_sin_f32(Cloud_Angle_Measure[YAW][MECH]);
		cos_yaw = arm_cos_f32(Cloud_Angle_Measure[YAW][MECH]);
#if YAW_POSITION == YAW_DOWN //�������¶�ȡ�ĵ���Ƕ�ֵ�Ƿ���
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][CHANGE];
#else
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][MECH];
#endif

		Chassis_Move_X = cos_yaw * Chassis_Move_X + sin_yaw * Chassis_Move_Y; //�����ǿ����ƶ�ʱ��Ҫ�õ�
		Chassis_Move_Y = -sin_yaw * Chassis_Move_X + cos_yaw * Chassis_Move_Y;

		//������תPID���ٶ�
		Chassis_Move_Z = PID_Calc(&chassis_angle_pid, C_A_M_Y_M, rad_format(angle_swing_set));
		//�ٶ��޷�
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}

	else if (Chassis_Mode == CHASSIS_GYRO_MODE)
	{
		Angle_error();
		//������ת�Ľ��ٶ�
		Chassis_Move_Z = -PID_Calc(&chassis_angle_key_pid, 0.0f, Chassis_Gyro_Error); //���赱ǰʵ��ֵΪ0���趨���ֵΪĿ��ֵ������趨��ǰĿ��ֵΪ0��ʵ��ֵΪ���ֵ����ʱ���ֵӦȡ�������˶�����ȡ����
		//���õ����˶����ٶ�
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_MECH_MODE)
	{
		Angle_error();
		Chassis_Move_Z = CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[0];
		Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_TOP_MODE)
	{
		if ((fabs(Chassis_Move_X) < 0.001) && (fabs(Chassis_Move_Y) < 0.001))
		{
			Chassis_Move_Z = 1.5f;
		}
		else
		{
			Chassis_Move_Z = 1.0f;
			Angle_error();

			Chassis_Move_X1 = (cos(theta) * Chassis_Move_X) + (-sin(theta) * Chassis_Move_Y);
			Chassis_Move_Y1 = (sin(theta) * Chassis_Move_X) + (cos(theta) * Chassis_Move_Y);
			Chassis_Move_X = fp32_constrain(Chassis_Move_X1, vx_min_speed, vx_max_speed);
			Chassis_Move_Y = fp32_constrain(Chassis_Move_Y1, vy_min_speed, vy_max_speed);
			Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
		}
	}
}

/**
  * @brief  ���̿��ƣ��ó�����ȫ���˶��ٶ�
  * @param  void
  * @retval void
  * @attention ģʽѡ��,����ĳģʽ��ǵ�д�˳�����ͨģʽ���ж�
  * �ް������»�һֱ�����Զ�����ģʽ,ģʽ�л��İ�����������ģʽ�л�ѡ��ģʽ
  */
void Chassis_Key_Ctrl(void)
{
	if (remote_change == TRUE) //�մ�ң��ģʽ�й���,Ĭ��Ϊ������ģʽ
	{
		Chassis_Mode = CHASSIS_GYRO_MODE;
		remote_change = FALSE;
	}
	Chassis_NORMAL_Mode_Ctrl(); //����ͨģʽ��ִ��ģʽ�л���Ҳ��������ʻ��ģʽ
	switch (Chassis_Action)		//ö������δ�������ᾯ��
	{
	case CHASSIS_NORMAL:			//��ͨģʽ
		Chassis_NORMAL_Mode_Ctrl(); //����ͨģʽ��ִ��ģʽ�л���Ҳ��������ʻ��ģʽ
		Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
		break;
		/*----------------------------С����ģʽ---------------------------	*/
	case CHASSIS_REVOLVE:
		if (!IF_KEY_PRESSED_F)
		{
			Chass_Switch_F = 1;
		}

		if (IF_KEY_PRESSED_F && Chass_Switch_F == 1)
		{
			Chass_Switch_F = 0;
			Chass_Key_F_Change++;
			Chass_Key_F_Change %= 2; //����������Ч��ż������Ч����һ�ο��ٰ�һ�ι�
		}

		if (Chass_Key_F_Change)
		{
			Chassis_Mode = CHASSIS_TOP_MODE; //������ģʽ,���̸�����̨��
			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
			//����Ҫдһ��С����ģʽ�ĺ���
		}
		else
		{
			Chassis_Action = CHASSIS_NORMAL; //�˳�С����ģʽ��������ͨģʽ
		}
		break;
		/*----------------------------����ģʽ---------------------------	*/
	case CHASSIS_UP:
		CHASSIS_UPUP_Mode_Ctrl();
		break;

		//		case CHASSIS_MISS:    //�Զ�����ģʽ
		//			CHASSIS_MISS_Mode_Ctrl();     //���ܲ�����
		//		break;
		/*----------------------------���ٲ���ģʽ----------------------------*/
	case CHASSIS_SLOW:
		if (Magazine_IfOpen() != TRUE) //���ֹر�
		{
			Chassis_Action = CHASSIS_NORMAL; //�����˳�����ģʽ
		}
		else
		{
			Chassis_Mode = CHASSIS_MECH_MODE; //����ʱ���̽����еģʽ

			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_SLOW, TIME_INC_SLOW);
			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_SLOW);
		}
		break;
		/*----------------------------���ģʽ----------------------------*/
	case CHASSIS_BUFF:
		if (GIMBAL_IfBuffHit() != TRUE)
		{
			Chassis_Action = CHASSIS_NORMAL;  //�˳����ģʽ
			Chassis_Mode = CHASSIS_GYRO_MODE; //�л�������ģʽ
		}
		else
		{
			Chassis_Mode = CHASSIS_MECH_MODE; //������̽����еģʽ
			CHASSIS_REST();					  //Ŀ���ٶ���0
		}
		break;
	}
}

/**
  * @brief  ���̼���ģʽѡ��,������Ӧ
  * @param  void
  * @retval void
  * @attention ���̼��̿���״̬�µ�����ģʽ�л�������
  */
void Chassis_NORMAL_Mode_Ctrl(void)
{
	/*----------------------------------------------****������ǰ����Ͱ�һ��****--------------------------------------------------------*/
	if (!IF_KEY_PRESSED_F) //F�ɿ�
	{
		Chass_Switch_F = 1;
	}

	if (!IF_KEY_PRESSED_F) //R�ɿ�
	{
		Chass_Switch_F = 1;
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*----------------------------------------------****F��һ�������Ťƨ��ģʽ****----------------------------------------------------*/
	//	if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL  && Chass_Switch_F == 1)
	//	{
	//		Chass_Switch_F = 0;
	//		Chass_Key_F_Change ++;
	//		Chass_Key_F_Change %= 2;
	//		Chassis_Action = CHASSIS_SHAKE;//�ǵ�д�����˳�Ťƨ��ģʽ�ĺ���
	//	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*----------------------------------------------****R��һ�������С����ģʽ****----------------------------------------------------*/

	else if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL && Chass_Switch_F == 1)
	{
		Chass_Switch_F = 0;
		Chass_Key_F_Change++;
		Chass_Key_F_Change %= 2;
		Chassis_Action = CHASSIS_REVOLVE; //�ǵ�д�����˳�С����ģʽ�ĺ���
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*---------------------------------------------****W��Ctrlһ�����������ģʽ****----------------------------------------------------*/

	else if (IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL)
	{
		Chassis_Action = CHASSIS_UP; //����ģʽ
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*---------------------------------------------****����ֻҪ�����ͽ�����ٲ���ģʽ****----------------------------------------------------*/
	else if (Magazine_IfOpen() == TRUE) //���ֿ���,���벹��ģʽ
	{
		Chassis_Action = CHASSIS_SLOW;
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*---------------------------------------------****ȷ�ϴ��ģʽ�ͽ�����ģʽ****----------------------------------------------------*/
	//����
	else if (GIMBAL_IfBuffHit() == TRUE) //���ģʽ
	{
		Chassis_Action = CHASSIS_BUFF;
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/
		/* 	��������û�б�Ҫ */
		/*-----------------------------------------------****��סCtrl������еģʽ****----------------------------------------------------*/
		// if (IF_KEY_PRESSED_CTRL)
		// {
		// 	Chassis_Mode = CHASSIS_MECH_MODE;
		// }
		// else //�ɿ�CTRL����������ģʽ
		// {
		// 	Chassis_Mode = CHASSIS_GYRO_MODE;
		// }
		// Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL); //�����ٶ����ֵ��б��ʱ��
		// Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);
}

/*-------------------------------------------�����̿��Ƽ���Chassis_Move_X |Chassis_Move_Y |Chassis_Move_Z-------------------------------------------------*/
/**
  * @brief  �����Ƶ�����ת,����QEC���ƿ���תȦ
  * @param  �ٶ��������� 
  * @retval void
  * @attention  ������������ת
  */

void Chassis_Mouse_Move_Calculate(int16_t sRevolMax)
{
	//	static int16_t sErrorPrev = 0;//�ϴ�ƫ�����
	float sErrorReal = 0; //yawƫ���������

	Chassis_Revolve_Move_Max = sRevolMax; //������ת����

	if (Chassis_Mode == CHASSIS_GYRO_MODE) //������ģʽ
	{
		sErrorReal = GIMBAL_GetOffsetAngle(); //��ȡʵʱƫ��,����Ťƨ���µĵ���λ�ò���
		Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f, sErrorReal);
		//Chassis_Move_Z = Chassis_SpeedZ_PID(sErrorReal, kKey_Gyro_Chassis_Revolve);
		//Chassis_Move_Z = PID_Calc(&chassis_angle_pid, 0.0f, Chassis_Gyro_Error);
	}
	else //��еģʽ
	{
		Chassis_Move_Z = constrain_float(MOUSE_X_MOVE_SPEED * kKey_Mech_Chassis_Revolve, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);
	}
}

/**
  * @brief  ����ģʽ�µ����˶�����
  * @param  �ٶ���������    �����ٶ�(���293)
  * @retval void
  * @attention  ���̿���ǰ������ƽ��,ƽ���޻�е��������ģʽ֮��
  *             ��Ҫ��ȡʱ��������б�º�������
  */
void Chassis_Keyboard_Move_Calculate(int16_t sMoveMax, int16_t sMoveRamp)
{
	static portTickType ulCurrentTime = 0;
	static uint32_t ulDelay = 0;
	float k_rc_z = 1; //����Z�ٶȵ���ǰ������ƽ�����ٱ�

	Chassis_Standard_Move_Max = sMoveMax; //�����ٶ��޷�,ˮƽ�ƶ�
	timeInc = sMoveRamp;

	ulCurrentTime = xTaskGetTickCount(); //��ǰϵͳʱ��

	if (fabs(Chassis_Move_Z) > 800) //Ťͷ�ٶ�Խ��,ǰ���ٶ�Խ��,��ֹת��뾶����
	{
		k_rc_z = ((Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800)) / (Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max);

		k_rc_z = constrain_float(k_rc_z, 0, 1);
	}
	else
	{
		k_rc_z = 1;
	}

	if (ulCurrentTime >= ulDelay) //ÿ10ms�仯һ��б����
	{
		ulDelay = ulCurrentTime + TIME_STAMP_10MS;

		if (Chassis_Action == CHASSIS_NORMAL && !KEY_PRESSED_OFFSET_SHIFT) //ֻ��һ��ģʽ�²��ж��ٶ�ͻ�����,��ֹ��
		{
			if (IF_KEY_PRESSED_W) //�ȳ������ݳ����ٲ��Ե��ݷŵ�ʱ�Ƿ�Ҫȫ������
			{
				timeXBack = 0; //����ǰ�������б�¹���,�����´μ������б��
				//ǰ��X������,���ڴ˼��볬�����ݰ����ж�
				if (Chassis_Move_X < sMoveMax / 2.5)		//ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{											//�����ٶ��Ƿ�����ٶȵ�1/5���жϲ�֪���Ƿ����
					timeInc_Saltation = TIME_INC_SALTATION; //������ģʽ�����ٶȷ���ͻ��
				}
				else //�Ѿ����˴�ʱ������������һ�����ٶ�
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0; //ͬ��
				//����X�Ǹ�
				if (Chassis_Move_X > (-sMoveMax) / 2.5) //ת��ͻ��,�տ�ʼ��һС��ʱ��б�½���,��ֹ���Ӵ��˷ѹ���
				{
					timeInc_Saltation = TIME_INC_SALTATION; //������ģʽ�����ٶȷ���ͻ��
				}
				else //�Ѿ����˴�ʱ������������һ�����ٶ�
				{
					timeInc_Saltation = sMoveRamp;
				}
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}

			//����ģʽ��ȫ���ƶ�,б��������,ע������,��������*б�±����õ��������ӵ�ֵ,ģ��ҡ��
			//ǰ�������б���Ǳ仯��
			Slope_Chassis_Move_Fron = (int16_t)(Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_D, &timeXFron, timeInc_Saltation, TIME_DEC_NORMAL));

			Slope_Chassis_Move_Back = (int16_t)(-Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_A, &timeXBack, timeInc_Saltation, TIME_DEC_NORMAL));

			//���ҵ�����б�¸�ǰ��һ��,����
			Slope_Chassis_Move_Left = (int16_t)(+Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_S, &timeYRigh, timeInc / 1.5, TIME_DEC_NORMAL));

			Slope_Chassis_Move_Righ = (int16_t)(Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_W, &timeYLeft, timeInc / 1.5, TIME_DEC_NORMAL));

			Chassis_Move_X = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z; //��ң������ֵת��Ϊ�������˶����ٶ�
			Chassis_Move_Y = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;
		}
		else //����ģʽ����Ҫ�����ٶȷ���ͻ�����⴦��
		{
			if (IF_KEY_PRESSED_W)
			{
				timeXBack = 0; //����ǰ�������б�¹���,�����´μ������б��
			}

			if (IF_KEY_PRESSED_S)
			{
				timeXFron = 0; //ͬ��
			}

			if (IF_KEY_PRESSED_D)
			{
				timeYRigh = 0;
			}

			if (IF_KEY_PRESSED_A)
			{
				timeYLeft = 0;
			}

			Slope_Chassis_Move_Fron = (int16_t)(-Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_D, &timeXFron, timeInc, TIME_DEC_NORMAL));

			Slope_Chassis_Move_Back = (int16_t)(+Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_A, &timeXBack, timeInc, TIME_DEC_NORMAL));

			Slope_Chassis_Move_Left = (int16_t)(-Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_S, &timeYRigh, timeInc, TIME_DEC_NORMAL));

			Slope_Chassis_Move_Righ = (int16_t)(+Chassis_Standard_Move_Max *
												Chassis_Key_MoveRamp(IF_KEY_PRESSED_W, &timeYLeft, timeInc, TIME_DEC_NORMAL));

			if (Chassis_Action != CHASSIS_SHAKE)
			{
				Chassis_Move_X = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z; //��ң������ֵת��Ϊ�������˶����ٶ�
				Chassis_Move_Y = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;

			}
		}
	}
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------����ģʽ��ض���ģʽ----------------------------------------------------------------------------------------*/
/*-------------------------------------------------Ť����С���ݣ�δд�����ֶ����£��Զ����ܣ����ٲ�����δд�������ģʽ��δд��----------------------------------------------*/
/**
  * @brief  Ťƨ��ģʽ(λ�ò����)
  * @param  �ٶ���������    ���ӵ����������ʱ��
  * @retval void
  * @attention  ����ʱ�䣬Ť��λ�˾ͻ���
  */
//Ťƨ�ɻ���ѡ��
#define CORGI_BEGIN 0
#define CORGI_LEFT 1
#define CORGI_RIGH 2

uint16_t stateCorgi = CORGI_BEGIN; //�������Ť,Ĭ�ϲ�Ť
bool IfCorgiChange = FALSE;		   //�Ƿ�Ť������һ��
float corgi_angle_target = 0;	   //����Ŀ��Ƕ�
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp)
{
	//	int16_t  sAngleError   = 0;
	//	float    vectorXBuffer = 0;
	//	float    vectorYBuffer = 0;
	//	float    angle         = 0;

	//	Chassis_Revolve_Move_Max = sRevolMax;//����ٶ�����
	//	Slope_Chassis_Revolve_Move = sRevolRamp;//Ťͷб������

	//	sAngleError = GIMBAL_GetOffsetAngle();//����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	////	angle = -(float)sAngleError / (float)4096 * PI;
	//	angle = -(float)sAngleError;

	//	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//�ݴ�ʵʱX�仯
	//	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	//	Chassis_Move_X = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	//	Chassis_Move_Y = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	//�ؼ�:��������......
	//	switch (stateCorgi)
	//	{
	//		case CORGI_BEGIN:	//�Ժ���������ø����(��־λ��ͣȡ��),���ÿ�ʼŤͷ�ķ������
	//			corgi_angle_target = -900;//�ɸ�����ƶ��Ƕ�,�Զ�����ģʽ�±�����ʱ��Ť���Ƕ�
	//			IfCorgiChange = FALSE;
	//			stateCorgi    = CORGI_LEFT;
	//		break;
	//
	//		case CORGI_LEFT:
	//			corgi_angle_target = -1024;//�ɸ�����ƶ��Ƕ�
	//			IfCorgiChange = FALSE;

	//			if (sAngleError < -700)//�Ƕ�������700
	//			{
	//					stateCorgi = CORGI_RIGH;
	//				  IfCorgiChange = TRUE;//��ǿ��Ի���
	//			}
	//		break;
	//
	//		case CORGI_RIGH:
	//			corgi_angle_target = 1024;
	//			IfCorgiChange = FALSE;

	//			if (sAngleError > 700)//�Ƕ�������700
	//			{
	//				stateCorgi = CORGI_LEFT;
	//				IfCorgiChange = TRUE;//��ǿ��Ի���
	//			}
	//		break;
	//	}
	//  corgi_angle_target = 0.3;

	//	Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

	float C_A_M_Y_M = 0;

	chassis_follow_gimbal_yaw_control(); //�Ƿ���Ҫҡ��      ������õ�angle_swing_set

	//��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
	sin_yaw = arm_sin_f32(Cloud_Angle_Measure[YAW][MECH]);
	cos_yaw = arm_cos_f32(Cloud_Angle_Measure[YAW][MECH]);
#if YAW_POSITION == YAW_DOWN //�������¶�ȡ�ĵ���Ƕ�ֵ�Ƿ���
	C_A_M_Y_M = Cloud_Angle_Measure[YAW][CHANGE];
#else
	C_A_M_Y_M = Cloud_Angle_Measure[YAW][MECH];
#endif
	Chassis_Move_X = cos_yaw * Chassis_Move_X + sin_yaw * Chassis_Move_Y; //�����ǿ����ƶ�ʱ��Ҫ�õ�
	Chassis_Move_Y = -sin_yaw * Chassis_Move_X + cos_yaw * Chassis_Move_Y;

	//������תPID���ٶ�
	Chassis_Move_Z = PID_Calc(&chassis_angle_pid, C_A_M_Y_M, rad_format(angle_swing_set));
	//�ٶ��޷�
	Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
	Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
}

/**
  * @brief  Ťƨ��ģʽ(ʱ�䲻���)
  * @param  �ٶ���������    ���ӵ����������ʱ��
  * @retval void
  * @attention  ����ŤûŤ��λ��ʱ�䵽�˾ͻ���
  */
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp)
{
	static uint32_t twist_count;
	static float chass_yaw_set;
	float angle = 0;
	int16_t sAngleError = 0;
	float vectorXBuffer = 0;
	float vectorYBuffer = 0;

	static int16_t twist_period = 800; //500*2msΪһ��Ť������
	static int16_t twist_angle = 40;   //����������ŷ���Ƕ�����

	twist_count++;

	Chassis_Revolve_Move_Max = sRevolMax;	 //����ٶ�����
	Slope_Chassis_Revolve_Move = sRevolRamp; //Ťͷб������

	sAngleError = GIMBAL_GetOffsetAngle(); //����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//����Ƕ�ƫ��,��е�Ƕ�ת����ŷ����,����ǰ���ٶȲ���
	angle = -(float)sAngleError / (float)8192 * 360; //6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron; //�ݴ�ʵʱX�仯
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	//���Һ���Ťƨ��
	Chassis_Move_X = vectorXBuffer * cos(angle / 57.3f) - vectorYBuffer * sin(angle / 57.3f);
	Chassis_Move_Y = vectorXBuffer * sin(angle / 57.3f) + vectorYBuffer * cos(angle / 57.3f);

	chass_yaw_set = twist_angle * sin(2 * PI / twist_period * twist_count); //�������Ŀ�����Ƕ�
	Chassis_Move_Z = -Chassis_Z_Corgi(angle, chass_yaw_set);
}

/**
  * @brief  Ťƨ��ר�ã�ʱ�䲻��棩
  * @param  ��ǰ��̨ƫ������Ŀ��ƫ����
  * @retval ��ת�ٶ�
  * @attention 
  */
float Chassis_Z_Corgi(float get, float set)
{
	float error[2];
	float z = 0;

	error[NOW] = set - get;

	z = 15 * ((error[NOW] * 8) + 10 * (error[NOW] - error[LAST])); //PD����
	z = constrain_float(z, -5000, 5000);

	error[LAST] = error[NOW];

	return z;
}

/**
  * @brief  �ֶ�����ģʽ
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_UPUP_Mode_Ctrl(void)
{
	if (!IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL) //�ɿ�����һ���˳�����ģʽ

	{
		Chassis_Action = CHASSIS_NORMAL; //�����˳�����ģʽ
	}
	else
	{
		Chassis_Mode = CHASSIS_GYRO_MODE; //������ģʽ

		Chassis_Keyboard_Move_Calculate(STANDARD_MAX_UPUP, TIME_INC_UPUP);
		Chassis_Mouse_Move_Calculate(REVOLVE_MAX_UPUP);
	}
}

/**
  * @brief  �Զ�����ģʽ
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_MISS_Mode_Ctrl(void)
{
	int16_t sAngleError = 0;
	sAngleError = GIMBAL_GetOffsetAngle(); //����yaw����ƫ��,��֤����Ťƨ�ɵ�ʱ��Ҳ�ܸ�����̨��

	//�а������»��߳�ʱ��û���ܵ��������˳��Զ�����,��֤��������
	if (IF_KEY_PRESSED || Miss_Mode_Time > MISS_MAX_TIME)
	{
		Chassis_Action = CHASSIS_NORMAL; //�����л�������ģʽ
		Miss_Mode_Time = 0;
	}
	else
	{
		Chassis_Mode = CHASSIS_GYRO_MODE;

		if (JUDGE_IfArmorHurt() == TRUE //װ�װ����ݸ���,���ܵ��µ��˺�
			|| IfCorgiChange == FALSE)	//ƨ��û��Ť���Ա�
		{
			//����Ťƨ��һ��
			CHASSIS_CORGI_Mode_Ctrl(REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
			Miss_Mode_Time = 0;
		}
		else
		{
			Slope_Chassis_Move_Z = 0; //Ťƨ��ʵʱ���б��
			Chassis_Move_X = 0;
			Chassis_Move_Y = 0;
			Chassis_Move_Z = Chassis_SpeedZ_PID((sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
			Miss_Mode_Time++;
		}
	}
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  ����ȫ���㷨,��������ת��
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  *              	Xǰ(+)��(-)     Y��(-)��(+)     ZŤͷ
  */
void Chassis_Omni_Move_Calculate(void)
{
	/* NEW */
	static float rotate_ratio_fl; //ǰ��
	static float rotate_ratio_fr; //ǰ��
	static float rotate_ratio_bl; //����
	static float rotate_ratio_br; //����
	static float wheel_rpm_ratio;

	rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_Y_OFFSET) / RADIAN_COEF;

	wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO); //	60/�ܳ�*������

	//ȫ���㷨

	Chassis_Speed_Target[LEFT_FRON_201] = (-Chassis_Move_X + Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_fr) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_FRON_202] = (-Chassis_Move_X - Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_fl) * wheel_rpm_ratio;
	Chassis_Speed_Target[LEFT_BACK_203] = (+Chassis_Move_X + Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_bl) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_BACK_204] = (+Chassis_Move_X - Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_br) * wheel_rpm_ratio;
}

/**
  * @brief  ���̵��PID�������
  * @param  ���ID
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_Motor_Speed_PID(void)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	uint8_t i = 0;

	//�������ӿ�������ٶȣ�������������ٶ�
	for (i = 0; i < 4; i++)
	{
		temp = fabs(Chassis_Speed_Target[i]);
		if (max_vector < temp)
		{
			max_vector = temp;
		}
	}

	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
		{
			Chassis_Speed_Target[i] *= vector_rate;
		}
	}

	//����pid

	for (i = 0; i < 4; i++)
	{
		PID_Calc(&motor_pid[i], motor_chassis_speed[i], Chassis_Speed_Target[i]);
	}

	//��ֵ����ֵ
	for (i = 0; i < 4; i++)
	{
		Chassis_Final_Output[i] = motor_pid[i].out;
	}
}

/**
  * @brief  ����ģʽ���̵��PID�������
  * @param  ���ID
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_Motor_Speed_PID_KEY(void)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	fp32 temp = 0.0f;
	uint8_t i = 0;

	//�������ӿ�������ٶȣ�������������ٶ�
	for (i = 0; i < 4; i++)
	{
		temp = fabs(Chassis_Speed_Target[i]);
		if (max_vector < temp)
		{
			max_vector = temp;
		}
	}
	//���̲�ͬ���ʿ��� 
	if (max_vector > MAX_WHEEL_SPEED && Chassis_Mode == LOW)
	{
		vector_rate = MAX_WHEEL_SPEED / (max_vector);
		for (i = 0; i < 4; i++)
		{
			Chassis_Speed_Target[i] *= vector_rate / 1.6;
		}
	}
	else if (max_vector > MAX_WHEEL_SPEED && Chassis_Mode == MID)
	{
		vector_rate = MAX_WHEEL_SPEED / (max_vector);
		for (i = 0; i < 4; i++)
		{
			Chassis_Speed_Target[i] *= vector_rate / 1.3;
		}
	}
	else if (max_vector > MAX_WHEEL_SPEED && Chassis_Mode == HIGH)
	{
		vector_rate = MAX_WHEEL_SPEED / (max_vector);
		for (i = 0; i < 4; i++)
		{
			Chassis_Speed_Target[i] *= vector_rate;
		}
	}



	//����pid

	for (i = 0; i < 4; i++)
	{
		PID_Calc(&motor_key_pid[i], motor_key_chassis_speed[i], Chassis_Speed_Target[i]);
	}

	//��ֵ����ֵ
	for (i = 0; i < 4; i++)
	{
		Chassis_Final_Output[i] = constrain_float(motor_key_pid[i].out, -4000, 4000);
	}
}

/**
  * @brief  ���͵�����յ���ֵ
  * @param  void
  * @retval void
  * @attention  CAN1����
  */
void CHASSIS_CANSend(void)
{
	CAN_CMD_CHASSIS(Chassis_Final_Output[0], Chassis_Final_Output[1], Chassis_Final_Output[2], Chassis_Final_Output[3]);
}

/**
  * @brief  ���̵��PID����,����
  * @param  ���ID
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_Motor_Key_PID(ChassisWheel Wheel)
{

	//�����ٶ����
	Chassis_Speed_Error[Wheel] = Chassis_Speed_Target[Wheel] - Chassis_Speed_Measure[Wheel];

	Chassis_Speed_Error_Sum[Wheel] += Chassis_Speed_Error[Wheel];

	pTermChassis[Wheel] = Chassis_Speed_Error[Wheel] * Chassis_Speed_kpid[Wheel][KP];
	iTermChassis[Wheel] = Chassis_Speed_Error_Sum[Wheel] * Chassis_Speed_kpid[Wheel][KI] * 0.002f;
	//�����޷�
	iTermChassis[Wheel] = constrain_float(iTermChassis[Wheel], -iTermChassis_Max, iTermChassis_Max);

	Chassis_Speed_Error_NOW[Wheel] = Chassis_Speed_Error[Wheel];
	dTermChassis[Wheel] = (Chassis_Speed_Error_NOW[Wheel] - Chassis_Speed_Error_LAST[Wheel]) * Chassis_Speed_kpid[Wheel][KD];
	Chassis_Speed_Error_LAST[Wheel] = Chassis_Speed_Error_NOW[Wheel];

	//���������С,��ֹ���Ϊ0ʱͻȻʧ��
	if (pTermChassis[Wheel] * iTermChassis[Wheel] < 0)
	{
		Chassis_Speed_Error_Sum[Wheel] = constrain_float(Chassis_Speed_Error_Sum[Wheel],
														 -(3000 / Chassis_Speed_kpid[Wheel][KI] / 5.f),
														 (3000 / Chassis_Speed_kpid[Wheel][KI] / 5.f));
	}

	pidTermChassis[Wheel] = pTermChassis[Wheel] + iTermChassis[Wheel] + dTermChassis[Wheel];

	pidTermChassis[Wheel] = constrain_float(pidTermChassis[Wheel], -Chassis_Final_Output_Max, Chassis_Final_Output_Max);

	//��¼�������
	Chassis_Final_Output[Wheel] = pidTermChassis[Wheel];
}

/**
  * @brief  ��ת�ٶ�PID����
  * @param  ��ǰ��̨ƫ������Ŀ��ƫ����
  * @retval ��ת�ٶ�
  * @attention 
  */
float speed_z_pterm = 0;
float speed_z_iterm = 0;
float speed_z_dterm = 0;
float speed_z = 0;
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp)
{
	static int16_t ErrorPrev = 0; //�ϴ�ƫ�����
	static int32_t ErrorSum = 0;  //�ϴ�ƫ�����
	static int32_t ErrorPR = 0;
	static int32_t ErrorPR_KF = 0;

	ErrorPR_KF = KalmanFilter(&Chassis_Error_Kalman, ErrorReal);

	//P
	speed_z_pterm = ErrorReal * kp; //����yawƫ�����ļ������
	speed_z_pterm = constrain_float(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);

	//I
	ErrorSum -= ErrorPR_KF;
	speed_z_iterm = ErrorSum * 3 * 0.002f;
	if (abs(ErrorReal) <= 10) //10
	{
		ErrorSum = 0;
	}
	//�����޷�
	speed_z_iterm = constrain_float(speed_z_iterm, -5000, 5000);

	//D
	ErrorPR = ErrorPR_KF - ErrorPrev;

	if (abs(ErrorPR_KF) > REVOLVE_ANGLE) //35
	{
		speed_z_dterm = -(ErrorPR)*REVOLVE_KD; //600;//650;//125.f;
	}
	else
	{
		speed_z_dterm = 0;
	}
	//Ťͷ����ٶ��޷�
	speed_z = speed_z_pterm + speed_z_dterm; // + speed_z_iterm;//// + ;
	speed_z = constrain(speed_z, -Chassis_Revolve_Move_Max, +Chassis_Revolve_Move_Max);

	ErrorPrev = ErrorPR_KF; //��¼�ϴ����

	return speed_z;
}

/**
  * @brief  �ֱ��4�������PID����,���������������(���͸������ֵ)
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�)
  */
void Chassis_MotorOutput(void)
{
	Chassis_Motor_Key_PID(LEFT_FRON_201);
	Chassis_Motor_Key_PID(RIGH_FRON_202);
	Chassis_Motor_Key_PID(LEFT_BACK_203);
	Chassis_Motor_Key_PID(RIGH_BACK_204);
}

/**
  * @brief  ���̼���б�º���
  * @param  �жϰ����Ƿ񱻰���, ʱ����, ÿ�����ӵ���, һ��Ҫ��С����
  * @retval б�±���ϵ��
  * @attention  0~1
  */
float Chassis_Key_MoveRamp(uint8_t status, int16_t *time, int16_t inc, int16_t dec)
{
	float factor = 0;

	factor = 0.15 * sqrt(0.15 * (*time)); //�����ٶ�б��,time�ۼӵ�296.3б�¾����  //0.15

	if (status == 1) //����������
	{
		if (factor < 1) //��ֹtime̫��
		{
			*time += inc;
		}
	}
	else //�����ɿ�
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}

	factor = constrain_float(factor, 0, 1); //ע��һ����float�����޷�

	return factor; //ע�ⷽ��
}

/**
  * @brief  ��ȡ�����ƶ�ģʽ
  * @param  void
  * @retval TRUE:��еģʽ    false:������ģʽ
  * @attention  
  */
bool CHASSIS_IfActiveMode(void)
{
	if (Chassis_Mode == CHASSIS_MECH_MODE)
	{
		return TRUE; //��е
	}
	else
	{
		return FALSE; //������
	}
}

/**
  * @brief  �����Ƿ���Ťƨ��ģʽ
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfSHAKE(void)
{
	if (Chassis_Action == CHASSIS_SHAKE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  ��ȡ����Ƕ�
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorAngle(ChassisWheel Wheel, int16_t angle)
{
	Chassis_Angle_Measure[Wheel] = angle;
}

/**
  * @brief  ��ȡ���ת��
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */

void CHASSIS_UpdateMotorSpeed(ChassisWheel Wheel, int16_t speed)
{
	Chassis_Speed_Measure[Wheel] = speed;
}

/**
  * @brief  ��ȡ���ת�ص���
  * @param  ID,CAN����
  * @retval void
  * @attention  (201/202/203/204 --> ��ǰ/��ǰ/���/�Һ�),CAN1�е���
  */
void CHASSIS_UpdateMotorCur(ChassisWheel Wheel, int16_t current)
{
	Chassis_Current_Measure[Wheel] = current;
}

void Angle_error(void)
{
	theta = -Cloud_Angle_Measure[YAW][MECH]; //+ theta_error;
	theta_error1 = cos(theta);
	theta_error2 = sin(theta);
}
