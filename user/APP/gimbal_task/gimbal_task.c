/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ����������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       AV  DV���ҵ�ͷ ���ֿ��� ��̨����ģʽ δ����   �������˲�����Ҫ�˽⣬���飬������ƻ���Ҫ�˽⡣
  * @history    ���ӳ��ڳ���   ���������볬��   �����Ҹ�    ������ڳ���  R�� �����Ҹ�    ���������Ҹ�       �����װ�Ļ�  ������   ����������
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Gimbal_Task.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "main.h"
#include "led.h"

#include "arm_math.h"

#include "user_lib.h"

#include "remote_control.h"
#include "CAN_Receive.h"

#include "pid.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "vision.h"
#include "magazine.h"
#include "chassis_task.h"
#include "gimbal_key_control.h"


#define gimbal_total_pid_clear(void)              \
    {                                             \
        Gimbal_PID_clear(&Gimbal_Yaw_Mech_PID);   \
        Gimbal_PID_clear(&Gimbal_Yaw_Gyro_PID);   \
        PID_clear(&gimbal_yaw_motor_gyro_pid);    \
        PID_clear(&gimbal_yaw_motor_mech_pid);    \
			                                      \
        Gimbal_PID_clear(&Gimbal_Pitch_Mech_PID); \
        Gimbal_PID_clear(&Gimbal_Pitch_Gyro_PID); \
        PID_clear(&gimbal_pitch_motor_gyro_pid);  \
        PID_clear(&gimbal_pitch_motor_mech_pid);  \
    }


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
/*--------------------------------------myself-------------------------------*/
extern  float Chassis_Gyro_Error ;
extern  RC_ctrl_t rc_ctrl;    //����ң�����ṹ�����
GimbalCtrlMode  modeGimbal;   //������̨����ģʽ    ��е/������
eGimbalAction  actGimbal;     //������̨�˶�ģʽ  ��ͷ ���� �����
Critical_t Yaw_Gyro_Angle;    
extKalman_t Vision_Distance_Kalman;                     //�����Ӿ����뿨�����˲��ṹ��
speed_calc_data_t Vision_Yaw_speed_Struct;              //�����Ӿ�yaw�ٶȲ��ٽṹ��
speed_calc_data_t Vision_Pitch_speed_Struct;            //�����Ӿ�pitch�ٶȲ��ٽṹ��
kalman_filter_t yaw_kalman_filter;                      //����yaw�������˲����ṹ��
kalman_filter_t pitch_kalman_filter;                    //����pitch�������˲����ṹ��

extern VisionRecvData_t VisionRecvData; //�����Ӿ����յ����ݽṹ��

Gimbal_PID_t Gimbal_Yaw_Mech_PID;      //PIDһϵ�нṹ��
Gimbal_PID_t Gimbal_Yaw_Gyro_PID;
Gimbal_PID_t Gimbal_Pitch_Mech_PID;
Gimbal_PID_t Gimbal_Pitch_Gyro_PID;
Gimbal_PID_t Gimbal_Pitch_key_PID;
Gimbal_PID_t Gimbal_yaw_key_PID;
PidTypeDef gimbal_yaw_motor_gyro_pid;
PidTypeDef gimbal_pitch_motor_gyro_pid;
PidTypeDef gimbal_yaw_motor_mech_pid;
PidTypeDef gimbal_pitch_motor_mech_pid;
PidTypeDef gumbal_angle_pid;
/*-----------------------------------------------------PID����----------------------------------------------------------------------------*/

//�����ǲ���
float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//�����ǽǶ�ֵ
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//�����ǽ��ٶ�ֵ

//��е�Ƕ��м����,��CAN�ж�ȡ����
int16_t  angleMotorPit,  angleMotorYaw; 
int16_t  speedMotorPit,  speedMotorYaw; 
int16_t  currentMotorPit,  currentMotorYaw; 
int16_t  out;
//�����Ƕ�
float Cloud_Angle_Target[2][3];//  pitch/yaw    mech/gyro
extern float Cloud_Angle_Target_GD[2][2];   //  pitch/yaw    mech/gyro  ������key_control.c��

//�����Ƕ�
float Cloud_Angle_Measure[2][4];//  pitch/yaw    mech/gyro/change      change���ڵ��̸���������ģʽ,��ֹ��ת180��

//�������ת��
float Cloud_Speed_Measure[2][2];//  pitch/yaw    mech/gyro

//�����������ֵ
float Cloud_Current_Measure[2][2];//  pitch/yaw    mech/gyro

//�������ٶ�
float Cloud_Palstance_Measure[2][3];//  pitch/yaw    mech/gyro


float motor_gyro_set[2][3];  //PID�����⻷��������ٶ��趨ֵ  pitch/yaw    mech/gyro
//float motor_gyro_set[2][2]; 

float current_set[2][3];      //PID�����ڻ���������  pitch/yaw    mech/gyro
//float current_set[2][2];

float given_current[2][3];     //PID���ո�ֵ����  pitch/yaw    mech/gyro
//float given_current[2][2];

/*------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------ң������ر���-------------------------------------------------------------------*/

//�ϵ�б�±���
float Slope_Begin_Pitch = 0.0020 ;  //���ϵ�ʱ�ƶ�����
float Slope_Begin_Yaw = 0.0020 ;

fp32 rc_add_yaw, rc_add_pit;       //ң��������
int16_t yaw_channel, pitch_channel; //ң�����м����
fp32 bias_angle;      //��ǰ�������Ƕ�
 
/*--------------------------------------------------------��̨����ģʽ�¸�С������������--------------------------------------------------------------------*/
//��ͷģʽ�Ƕ�Ŀ��
extern float TURNMode_Yaw_Back_Total;//����C,yaw��Ҫ�ı�ĽǶ�ֵ
extern float TURNMode_Yaw_Turn_Total;//����QE,yaw��Ҫ�ı�ĽǶ�ֵ,������������ת

//����������ģʽ�����ͳ��yawƫ����,��ֵ���Լ�������С,��ֹ˦ͷ����
float Mouse_Gyro_Yaw, Mouse_Gyro_Pitch;

/*�Զ����õ�һЩ��־λ*/
bool Mobility_Prediction_Yaw = FALSE;//Ԥ���Ƿ�����־λ
bool Mobi_Pre_Yaw_Fire = FALSE;//Ĭ��Ԥ��û��λ����ֹ��ǹ

uint16_t mobpre_yaw_left_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_right_delay = 0;//����Ԥ����ʱ�жϿɿ�������
uint16_t mobpre_yaw_stop_delay = 0;//Ԥ��ر���ʱ�жϿɿ�������

//�ϲ�����Ԥ����
float pitch_angle_raw;
float yaw_angle_raw;
float Auto_Error_Pitch[2];
float Auto_Error_Yaw[2];
float Auto_Distance;
uint32_t Gimbal_Vision_Time[2];

//�²�����Ԥ����
float Auto_Distance;//Ԥ�����
float vision_time_update_time;
float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//�������˲��ٶȲ���ֵ
float *yaw_kf_result, *pitch_kf_result;//���׿������˲����,0�Ƕ� 1�ٶ�
float yaw_speed_k = 0;//yaw�ٶ�Ԥ�����
float kf_yaw_angcon = 0;//yawԤ������޷�
float pitch_speed_k = 0;//pitch�ٶ�Ԥ�����
float kf_pitch_angcon = 0;//pitchԤ������޷�
float debug_kf_y_angle;//yawԤ���ݴ�
float debug_kf_p_angle;//pitchԤ���ݴ�
float debug_kf_angle_temp;//Ԥ��Ƕ�б���ݴ���
float debug_kf_angle_ramp = 20;//Ԥ��Ƕ�б�±仯��
float kf_speed_yl = 0;//�ٶȹ��͹ر�Ԥ��
uint16_t Auto_KF_Delay = 0;//����ͻȻ����,�������˲�������ʱ
float debug_y_sk;// = 38;//35;//30;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float debug_y_sb_sk;//�ڱ�Ԥ��ϵ��
float debug_y_sb_brig_sk;//��ͷ�ڱ�
float debug_p_sk;//�ƶ�Ԥ��ϵ��,Խ��Ԥ��Խ��
float debug_auto_err_y=120;// = 10;//15;//10;//15;//yaw�Ƕȹ���ر�Ԥ��              ����ֵ�ڳ����л���Ҫ�޸�               
float debug_auto_err_p;//pitch�Ƕȹ���ر�Ԥ��
float debug_kf_delay=80;// = 150;//100;//200;//120;//150;//Ԥ����ʱ����              ����ֵ�ڳ����л���Ҫ�޸�
float debug_kf_speed_yl;//yaw�ٶȹ��͹ر�Ԥ��
float debug_kf_speed_yl_sb;//̧ͷ���ڱ�ʱ��С��Ϳɿ�Ԥ����
float debug_kf_speed_yh;//yaw�ٶȹ��߹ر�Ԥ��
float debug_kf_speed_pl;//pitch�ٶȹ��͹ر�Ԥ��
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yawԤ�����޷�
float debug_kf_p_angcon;//pitchԤ�����޷�



int my_op = 0;
/*------------------------------------------------------------------------------------------------------------------------------------------*/



//ÿ2msִ��һ��������
void GIMBAL_task(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{
		currentTime = xTaskGetTickCount();//��ǰϵͳʱ��
		
		/* ����� */
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//��ʼ��ģʽ
		{
       GIMBAL_InitCtrl();
		}
		else
		{
			if(IF_RC_SW2_UP)
			{
				if (IF_MOUSE_PRESSED_RIGH)
				{
					actGimbal = GIMBAL_AUTO;
				}
				KEY_Set_Mode();
			}
			else
			{
				RC_Set_Mode();
				GIMBAL_Set_Control();
			  actGimbal = GIMBAL_NORMAL;
			}
		}
		//���ݲ���ģʽ�任PID,ÿ�ζ�Ҫ��,����Ҫ
		//GIMBAL_PositionLoop();
		// if (IF_MOUSE_PRESSED_RIGH)
		// {
		// 	GIMBAL_PositionLoop_AUTO();
		// }
		// else
		// {
		// 	GIMBAL_PositionLoop();
		// }
		GIMBAL_CanSend();
		
		//�����������ģʽ,�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�      ????????????????????????????????
		
		if(VisionRecvData.distance >= 999.0f)
		{
			led_red_on();
		}
		else 
			
		{
			led_red_off();
		}

		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//������ʱ
	}
}


/**
  * @brief  ��̨��ʼ��,��Ҫ��PID��ʼ��
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_InitCtrl(void)
{
	
		static bool bAngleRecord  = FALSE;
	  static portTickType  ulTimeCurrent = 0;
	
	  if (xTaskGetTickCount( ) - ulTimeCurrent > TIME_STAMP_100MS)//��֤���ϵ�������´ο���
	   {
		   bAngleRecord = FALSE;
	   }
		 
		 ulTimeCurrent = xTaskGetTickCount( );
		 
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3]   = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		
		modeGimbal = CLOUD_MECH_MODE;
    //��ʼ��yaw���pid		
		GIMBAL_PID_Init(&Gimbal_Yaw_Gyro_PID,YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Yaw_Mech_PID,YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_yaw_key_PID,YAW_KEY_PID_MAX_OUT, YAW_KEY_PID_MAX_IOUT, YAW_KEY_PID_KP, YAW_KEY_PID_KI, YAW_KEY_PID_KD);
		PID_Init(&gimbal_yaw_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_yaw_motor_mech_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);		
		
    //��ʼ��pitch���pid	
		GIMBAL_PID_Init(&Gimbal_Pitch_Gyro_PID,PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Pitch_Mech_PID,PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Pitch_key_PID,PITCH_KEY_PID_MAX_OUT, PITCH_KEY_PID_MAX_IOUT, PITCH_KEY_PID_KP, PITCH_KEY_PID_KI, PITCH_KEY_PID_KD);
		PID_Init(&gimbal_pitch_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_pitch_motor_mech_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);		
		
		gimbal_total_pid_clear();		
		
		//��ʼ����̨����PID
		const static fp32 gimbal_yaw_pid[3] = {GIMBAL_FOLLOW_CHASSIS_PID_KP, GIMBAL_FOLLOW_CHASSIS_PID_KI, GIMBAL_FOLLOW_CHASSIS_PID_KD};          //��ʼ��������ת����KP��KI��KD 
    PID_Init(&gumbal_angle_pid, PID_POSITION, gimbal_yaw_pid, GIMBAL_FOLLOW_CHASSIS_PID_MAX_OUT, GIMBAL_FOLLOW_CHASSIS_PID_MAX_OUT);
		
		
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		
		Cloud_Angle_Target[YAW][TOP]=Cloud_Angle_Measure[YAW][GYRO];
		
			//��¼�ϵ�ʱ��̨��е�Ƕ�
	  if (bAngleRecord == FALSE)
	  {
	  	bAngleRecord = TRUE;
			
		  Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
		  Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
	  }
		
			//ƽ��������̨�ƶ����м�,��ֹ���ϵ��˦
	  Cloud_Angle_Target[PITCH][MECH] = RAMP_float( mid_pitch_angle, Cloud_Angle_Target[PITCH][MECH], Slope_Begin_Pitch);
	  Cloud_Angle_Target[YAW][MECH]   = RAMP_float( mid_yaw_angle, Cloud_Angle_Target[YAW][MECH], Slope_Begin_Yaw);
}
/*-----------------------------------------------��̨ң��������ģʽѡ���ң����Ŀ��ֵ����--------------------------------------------------*/
/**
  * @brief  ��̨ң��������ģʽ
  * @param  void
  * @retval void
  * @attention 
  */
void RC_Set_Mode(void)
{
	if(IF_RC_SW2_MID)
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}
	else  if(IF_RC_SW2_UP)
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}
	else if(IF_RC_SW2_DOWN)
	{
		modeGimbal = CLOUD_TOP_MODE;
	}
}

/**
  * @brief  ������̨��Ŀ��ֵ
  * @param  void
  * @retval void
  * @attention 
  */

void GIMBAL_Set_Control(void)
{
	  //��ң���������ݴ������� int16_t yaw_channel,pitch_channel
    rc_deadline_limit(RC_CH0_RLR_OFFSET, yaw_channel, RC_deadband);
    rc_deadline_limit(RC_CH1_RUD_OFFSET, pitch_channel, RC_deadband);	
	
    rc_add_yaw = yaw_channel * Yaw_RC_SEN ;
    rc_add_pit = - pitch_channel * Pitch_RC_SEN ;
	
	if(modeGimbal == CLOUD_MECH_MODE)
	{
		TOP_Rc_Switch();
#if YAW_POSITION ==	YAW_DOWN
//		Cloud_Angle_Target[YAW][MECH] -= (rc_add_yaw/5);                                 //������̨��װ��ʽ,���Ϻ�����������ͬ ,����ֵ��֤�ۼӵ�ֵΪ0.000001       /-
		
		
//		if( my_op == 10)	
//		{
//			Cloud_Angle_Target[YAW][MECH] = mid_yaw_angle;
//			my_op = 1;	
//		}
//		else
//		{
//			Cloud_Angle_Target[YAW][MECH] -= Chassis_Gyro_Error;
//			++my_op ;	
//		}
		
		
//		Cloud_Angle_Target[YAW][MECH] = mid_yaw_angle;
		Cloud_Angle_Target[YAW][MECH] =	PID_Calc(&gumbal_angle_pid,Chassis_Gyro_Error, Chassis_Gyro_Error);
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
    //�Ƿ񳬹���� ��Сֵ
    
    if (Cloud_Angle_Target[YAW][MECH] > max_yaw_relative_angle)                   //����Ĭ��ֱ����������
    {
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
    }
    else if (Cloud_Angle_Target[YAW][MECH] < min_yaw_relative_angle)
    {
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
    }
			
		
		Cloud_Angle_Target[PITCH][MECH]+= rc_add_pit;
		                               //ע������ jj
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //�Ƿ񳬹���� ��Сֵ
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }

#else 
//		Cloud_Angle_Target[YAW][MECH] += (rc_add_yaw/5);                                 //������̨��װ��ʽ,���Ϻ�����������ͬ        /-
		Cloud_Angle_Target[YAW][MECH] = mid_yaw_angle;
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
    //�Ƿ񳬹���� ��Сֵ
    
    if (Cloud_Angle_Target[YAW][MECH] > max_yaw_relative_angle)                   //����Ĭ��ֱ����������
    {
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
    }
    else if (Cloud_Angle_Target[YAW][MECH] < min_yaw_relative_angle)
    {
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
    }
			
		
		Cloud_Angle_Target[PITCH][MECH]-= rc_add_pit;
		                               //ע������                /+
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //�Ƿ񳬹���� ��Сֵ
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }
		
#endif 	
	}
	else if(modeGimbal == CLOUD_GYRO_MODE)
	{
		TOP_Rc_Switch();
//		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
//		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][MECH];
		
    Cloud_Angle_Target[YAW][GYRO] += rc_add_yaw;                                     //ע������
    Cloud_Angle_Target[PITCH][MECH] += rc_add_pit * 1.5;
		#if YAW_POSITION ==	YAW_DOWN
    
		Cloud_Angle_Target[PITCH][MECH]+= rc_add_pit;
		                               //ע������ jj
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //�Ƿ񳬹���� ��Сֵ
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }

#else 
		Cloud_Angle_Target[YAW][MECH] += (rc_add_yaw/5);                                 //������̨��װ��ʽ,���Ϻ�����������ͬ        /-
		
		Cloud_Angle_Target[PITCH][MECH]-= rc_add_pit;
		                               //ע������                /+
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //�Ƿ񳬹���� ��Сֵ
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }
		
#endif 	
	}
	
	else if(modeGimbal == CLOUD_TOP_MODE)
	{
		//Ӧ��Ҫ������ӵ���ǹ�ڵĳ���
		if(TOP_Rc_Switch() == TRUE)
		{
			Cloud_Angle_Target[YAW][TOP]=Cloud_Angle_Measure[YAW][GYRO];
			Cloud_Angle_Target[PITCH][MECH]=Cloud_Angle_Measure[PITCH][MECH];
		}
    Cloud_Angle_Target[YAW][TOP] += rc_add_yaw;                                     //ע������
    Cloud_Angle_Target[PITCH][MECH] += rc_add_pit;
//		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
//		Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
		Cloud_Angle_Target[YAW][GYRO]=Cloud_Angle_Measure[YAW][GYRO];
	}
}

/*-----------------------------------------------��̨���̿���ģʽѡ��ͼ���Ŀ��ֵ����--------------------------------------------------*/
/**
  * @brief  ��̨���̿���ģʽ
  * @param  void
  * @retval void
  * @attention 
  */
void KEY_Set_Mode(void)
{
	switch(actGimbal)
	{
		/*--------------��̨ģʽѡ��----------------*/
		case GIMBAL_NORMAL:
			GIMBAL_NORMAL_Mode_Ctrl();//�ڴ�ѡ�����ģʽ		  
		break;	

		/*--------------V  180���ͷ----------------*/
		case GIMBAL_AROUND:
			modeGimbal = CLOUD_GYRO_MODE;//����������ģʽ
		
			if (TURNMode_Yaw_Back_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Back_Total, Cloud_Angle_Target[YAW][GYRO], TurnSpeed );
			}
		break;

		
		/*------------���ֿ���,��ֹ̧ͷ-----------------*/
		case GIMBAL_LEVEL:
			GIMBAL_LEVEL_Mode_Ctrl();
		    Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Target_GD[YAW][MECH];
		    Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Target_GD[PITCH][MECH];
		break;
		
		/*--------------Q E  90���ͷ----------------*/
		case GIMBAL_TURN:				
			modeGimbal = CLOUD_GYRO_MODE;//����������ģʽ

		  if (TURNMode_Yaw_Turn_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Turn_Total, Cloud_Angle_Target[YAW][GYRO], TurnSpeed );
			}
		break;			
			
		case GIMBAL_AUTO:
			modeGimbal = CLOUD_GYRO_MODE;  //����������ģʽ
		  if(!IF_MOUSE_PRESSED_RIGH)  //�ɿ��Ҽ��˳�����ģʽ
			{
				actGimbal = GIMBAL_NORMAL;
				
				//����Ŀ��ƫ������,�����л�ʱ��̨����
				VisionRecvData.identify_target = FALSE;
				Auto_KF_Delay = 0;//������´��ӳ�Ԥ����
				Mobility_Prediction_Yaw = FALSE;//���Ԥ��û����
				Mobi_Pre_Yaw_Fire = FALSE;//Ĭ�ϱ��Ԥ��û��λ����ֹ����
				
				mobpre_yaw_left_delay  = 0;//������Ԥ��Ŀ����ӳ�
				mobpre_yaw_right_delay = 0;//������Ԥ��Ŀ����ӳ�	
				mobpre_yaw_stop_delay = 0;//ֹͣԤ�⿪����ʱ����
				
//				Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
//				Cloud_Angle_Measure[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
			}
			else if(IF_MOUSE_PRESSED_RIGH)
			{
				GIMBAL_AUTO_Mode_Ctrl();
				
			}
		break;
	}
}

/**
  * @brief  ��̨����ģʽ��Ŀ��ֵ����
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_Set_Key_Control(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//��겻����������Ӧ
	static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������Ӧ
	
	if(modeGimbal == CLOUD_MECH_MODE)
	{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;
			Cloud_Angle_Target[YAW][MECH]   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;	    //yaw���ֲ���,��Զ���м�
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];		
	}
	else if(modeGimbal == CLOUD_GYRO_MODE)
	{
      Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;//��¼Ŀ��仯�Ƕ�
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;//pitch�Ծ�ʹ�û�еģʽ
/*-------��곤ʱ�䲻������ֹ̨ͣ�ƶ�------*/
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
/*-----------------------------------------*/
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Yaw_Mouse_ramp );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Pitch_Mouse_ramp );		
	}
}

void GIMBAL_AUTO_Mode_Ctrl(void)//����+   ����-
{
	static float pitch_angle_ref;
	static float yaw_angle_ref;
	
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	if(Vision_If_Update() == TRUE)                    //���ݸ���
	{
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][MECH]-Auto_Error_Pitch[NOW]);
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][GYRO]+Auto_Error_Yaw[NOW]);
		Vision_Clean_Update_Flag();//����,�����һֱִ��
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//��ȡ�����ݵ�����ʱ��
	}
//	if(Gimbal_Vision_Time[NOW] != Gimbal_Vision_Time[LAST])                  //���¿������˲�����ֵ
//	{
//		pitch_angle_raw = pitch_angle_ref;
//		yaw_angle_raw = yaw_angle_ref;
//		Gimbal_Vision_Time[LAST] = Gimbal_Vision_Time[NOW];
//	}
	
//	if(VisionRecvData.identify_target == TRUE)                     //ʶ����Ŀ��
//	{
		Cloud_Angle_Target[YAW][GYRO] = (Cloud_Angle_Measure[YAW][MECH] - Auto_Error_Yaw[NOW]);
		Cloud_Angle_Target[PITCH][MECH] = (Cloud_Angle_Measure[PITCH][MECH] + Auto_Error_Pitch[NOW]);
//	}
	//float temp = Auto_Error_Pitch[NOW];
	//modeGimbal = CLOUD_GYRO_MODE;
}


void GIMBAL_AUTO_PREDICT_Mode_Ctrl()
{
	static uint32_t Mouse_Yaw_Stop  = 0;//��겻����������Ӧ
	static uint32_t Mouse_Pitch_Stop  = 0;//��겻����������Ӧ
	
	static float yaw_angle_raw, pitch_angle_raw;//�������˲��ǶȲ���ֵ
	static float yaw_angle_ref;//��¼Ŀ��Ƕ�
	static float pitch_angle_ref;//��¼Ŀ��Ƕ�
	
	float kf_delay_open = 0;
	
	Mobility_Prediction_Yaw = FALSE;
	Mobi_Pre_Yaw_Fire = FALSE;
	
	//��ȡ�Ƕ�ƫ����,�Ѿ�ת��Ϊ��������
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	
	Auto_Distance = KalmanFilter(&Vision_Distance_Kalman,Auto_Distance);
	
	kf_delay_open = debug_kf_delay;
	
	if(Vision_If_Update() == TRUE)                    //���ݸ���
	{
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][GYRO]+Auto_Error_Pitch[NOW]);//�õ��ĽǶ������������Ҫ�Ŵ���߼��ϲ���
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][GYRO]+Auto_Error_Yaw[NOW]);//�õ��ĽǶ������������Ҫ�Ŵ���߼��ϲ���
		Vision_Clean_Update_Flag();//����,�����һֱִ��
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//��ȡ�����ݵ�����ʱ��
	}
	if(Gimbal_Vision_Time[NOW] != Gimbal_Vision_Time[LAST])                  //���¿������˲�����ֵ
	{
		vision_time_update_time = Gimbal_Vision_Time[NOW] - Gimbal_Vision_Time[LAST];//�����Ӿ��ӳ�
		pitch_angle_raw = pitch_angle_ref;//���¶��׿������˲�����ֵ
		yaw_angle_raw = yaw_angle_ref;
		Gimbal_Vision_Time[LAST] = Gimbal_Vision_Time[NOW];
	}
	
	if(VisionRecvData.identify_target == TRUE)                     //ʶ����Ŀ��
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct,Gimbal_Vision_Time[NOW],yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct,Gimbal_Vision_Time[NOW],pitch_angle_raw);
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,yaw_angle_raw,Vision_Angle_Speed_Yaw);//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,pitch_angle_raw,Vision_Angle_Speed_Pitch);
		
		Auto_KF_Delay++;//�˲��ӳٿ���
		
		//Ŀ��������ʱ��СԤ��                              ����ûд!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		//�Ƿ�����׼�ڱ�,�趨ֵ��ͬ                           ����ûд!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		yaw_speed_k = debug_y_sk; 
		kf_yaw_angcon = debug_kf_y_angcon;
		kf_speed_yl = debug_kf_speed_yl;
		
		//Ť��ģʽ�µ��ӳ�Ԥ����Ҫ�ı�                             ����ûд!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	  if(fabs(Auto_Error_Yaw[NOW])<debug_auto_err_y&&Auto_KF_Delay > kf_delay_open && fabs(yaw_kf_result[1]) > kf_speed_yl && fabs(pitch_kf_result[1]) < debug_kf_speed_yh)//Ԥ�⿪������
	  {
			if(yaw_kf_result[1] >= 0)
	  	{
	  		debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[1] - kf_speed_yl) * 1;
	  	}
			
			else if(yaw_kf_result[1] <0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[1] + kf_speed_yl) * 1;
			}
			
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp,-debug_kf_y_angcon,debug_kf_y_angcon);//Ԥ���ݴ����޷�
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp,debug_kf_y_angle,debug_kf_angle_ramp);//Ԥ���������仯
			debug_kf_y_angle = constrain_float(debug_kf_y_angle,-debug_kf_y_angcon,debug_kf_y_angcon);
		  Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[0]+debug_kf_y_angle;
			
			if((yaw_kf_result[1]>0) && (Auto_Error_Yaw[NOW]<0.3f)) //���������Ƚϻ�����Ҫdebug��
			{
				mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_left_delay++;
				
				if(mobpre_yaw_left_delay > 0)//������ʱʱ��
				{
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ,���Կ���
				}
				else 
				{
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ�ⲻ��λ
				}
			}
			
			else if((yaw_kf_result[1]<0) && (Auto_Error_Yaw[NOW]>-0.3f))//���������Ƚϻ�����Ҫdebug��
			{
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_right_delay++;
				
				if(mobpre_yaw_right_delay > 0)//������ʱʱ���������
				{
					Mobi_Pre_Yaw_Fire = TRUE;//Ԥ�⵽λ,���Կ���
				}
				else 
				{
					Mobi_Pre_Yaw_Fire = FALSE;//Ԥ�ⲻ��λ
				}
			}
			
	  	else
		  {
				Mobi_Pre_Yaw_Fire = FALSE;//Ԥ�ⲻ��λ
				
				mobpre_yaw_left_delay = 0;//����Ԥ�⿪����ʱ����
				mobpre_yaw_right_delay = 0;//����Ԥ�⿪����ʱ����
	  	}
			
			Mobility_Prediction_Yaw = TRUE;
			mobpre_yaw_stop_delay = 0;
			

//				pitch_speed_k = debug_p_sk/2.f;
//		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
//        debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[1] + debug_kf_speed_pl);

		//		Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
	  }
		
		if(Auto_KF_Delay > debug_kf_delay && fabs(Auto_Error_Pitch[NOW]) > debug_auto_err_p && fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_pl && VisionRecvData.distance/100 < 4.f)
		{
		  if(VisionRecvData.auto_too_close == TRUE)
		  {
		  	pitch_speed_k = debug_p_sk/2.f;
		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
		  }
			
			if(pitch_kf_result[KF_SPEED]>=0)
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - debug_kf_speed_pl);
			}
			
			else 
			{
				pitch_speed_k = debug_p_sk;
				kf_pitch_angcon = debug_kf_p_angcon;
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + debug_kf_speed_pl);
			}
			
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);//Pitch�޷�
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;
		}
		else
		{
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
		}
	}
	
	else  //δʶ��Ŀ��
	{
		//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
//		//�ԽǶȺ��ٶȽ��ж��׿������˲��ں�,0λ��,1�ٶ�
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		debug_kf_angle_temp = 0;
		
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,Cloud_Angle_Measure[YAW][GYRO],0);
    pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,Cloud_Angle_Measure[PITCH][MECH],0);		

		if(modeGimbal == CLOUD_MECH_MODE)
	  {
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;
			Cloud_Angle_Target[YAW][MECH]   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;	    //yaw���ֲ���,��Զ���м�
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];		
	  }
	  else if(modeGimbal == CLOUD_GYRO_MODE)
	  {
      Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;//��¼Ŀ��仯�Ƕ�
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;//pitch�Ծ�ʹ�û�еģʽ
/*-------��곤ʱ�䲻������ֹ̨ͣ�ƶ�------*/
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//��곤ʱ��ͣ����ֹͣ�ƶ�
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
/*-----------------------------------------*/
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Yaw_Mouse_ramp );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Pitch_Mouse_ramp );		
	  }
		
		Auto_KF_Delay = 0;
		
		//Ԥ���ӳ�����
	}
}

/*-------------------------------------------------------PID�ܼ���������---------------------------------------------------------------*/
/**
  * @brief  pid����
  * @param  void
  * @retval void
  * @attention �˴����ܸı�Ŀ��Ƕ�,ֻ���������޷��͵���PID���㺯��
  */
void GIMBAL_PositionLoop(void)
{
  if(modeGimbal == CLOUD_MECH_MODE)
	{
		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/100;
		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/100;
		motor_gyro_set[YAW][MECH] = GIMBAL_PID_Calc(&Gimbal_Yaw_Mech_PID,Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][MECH],Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][MECH] = PID_Calc(&gimbal_yaw_motor_mech_pid,Cloud_Palstance_Measure[YAW][MECH],motor_gyro_set[YAW][MECH]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
    
    given_current[YAW][MECH]	=	 current_set[YAW][MECH];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_GYRO_MODE)
	{
//		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/10;
//		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/10;
		
		motor_gyro_set[YAW][GYRO] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID,Cloud_Angle_Measure[YAW][GYRO], Cloud_Angle_Target[YAW][GYRO],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
//		motor_gyro_set[PITCH][GYRO] = GIMBAL_PID_Calc(&Gimbal_Pitch_Gyro_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][GYRO],Cloud_Palstance_Measure[PITCH][GYRO]); 
		
		current_set[YAW][GYRO] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][GYRO]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
//		current_set[PITCH][GYRO] = PID_Calc(&gimbal_pitch_motor_gyro_pid,Cloud_Palstance_Measure[PITCH][GYRO],motor_gyro_set[PITCH][GYRO]);	
    
    given_current[YAW][GYRO]	=	 current_set[YAW][GYRO]; 
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_TOP_MODE)
	{
		motor_gyro_set[YAW][TOP] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID,Cloud_Angle_Measure[YAW][GYRO], Cloud_Angle_Target[YAW][TOP],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		
		current_set[YAW][TOP] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][TOP]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
		
		given_current[YAW][TOP]	=	 current_set[YAW][TOP];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
}


/**
  * @brief  ����pid����
  * @param  void
  * @retval void
  * @attention �˴����ܸı�Ŀ��Ƕ�,ֻ���������޷��͵���PID���㺯��
  */
void GIMBAL_PositionLoop_AUTO(void)
{
  if(modeGimbal == CLOUD_MECH_MODE)
	{
		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/100;
		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/100;
		motor_gyro_set[YAW][MECH] = GIMBAL_PID_Calc(&Gimbal_yaw_key_PID,Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][MECH],Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_key_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][MECH] = PID_Calc(&gimbal_yaw_motor_mech_pid,Cloud_Palstance_Measure[YAW][MECH],motor_gyro_set[YAW][MECH]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
    
    given_current[YAW][MECH]	=	 current_set[YAW][MECH];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_GYRO_MODE)
	{
//		Cloud_Palstance_Measure[YAW][GYRO] = Cloud_Palstance_Measure[YAW][MECH]/100;
//		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/10;
		motor_gyro_set[YAW][GYRO] = GIMBAL_PID_Calc(&Gimbal_yaw_key_PID,Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][GYRO],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_key_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
//		motor_gyro_set[PITCH][GYRO] = GIMBAL_PID_Calc(&Gimbal_Pitch_Gyro_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][GYRO],Cloud_Palstance_Measure[PITCH][GYRO]); 
		
		current_set[YAW][GYRO] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][GYRO]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
//		current_set[PITCH][GYRO] = PID_Calc(&gimbal_pitch_motor_gyro_pid,Cloud_Palstance_Measure[PITCH][GYRO],motor_gyro_set[PITCH][GYRO]);	
    
    given_current[YAW][GYRO]	=	 current_set[YAW][GYRO]; 
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_TOP_MODE)
	{
		motor_gyro_set[YAW][TOP] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID,Cloud_Angle_Measure[YAW][GYRO], Cloud_Angle_Target[YAW][TOP],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		
		current_set[YAW][TOP] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][TOP]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
		
		given_current[YAW][TOP]	=	 current_set[YAW][TOP];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
}

/*-------------------------------------------------------�������ͺ���������---------------------------------------------------------------*/
void GIMBAL_CanSend(void)
{
	float fMotorOutput[2] = {0};
		
	if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = given_current[YAW][MECH];
			fMotorOutput[PITCH] = given_current[PITCH][MECH];
		}
	else if(modeGimbal == CLOUD_GYRO_MODE)
		{
			
			fMotorOutput[YAW]   = given_current[YAW][GYRO] * (-YAW_POSITION);
			fMotorOutput[PITCH] = given_current[PITCH][MECH];
		}
		else if(modeGimbal == CLOUD_TOP_MODE)
	{
		fMotorOutput[YAW]   = given_current[YAW][TOP] * (-YAW_POSITION);
		fMotorOutput[PITCH] = given_current[PITCH][MECH];
	}
		
		
		CAN_CMD_GIMBAL(fMotorOutput[YAW],fMotorOutput[PITCH],0,0);
//		CAN_CMD_GIMBAL(1000,1000,0,0);
}





/*------------------------------------------------------------��������------------------------------------------------------------*/

/*        �ٽ�ֵ�ṹ���ʼ��    ��ȡ�����ǽǶȣ����ٶ�    Ŀ���ٶȼ��㺯��   YAW��ƫ�����ĽǶ�    ������̨����̷���Ƕ�  ����          */
/**
  * @brief �ٽ�ֵ�ṹ���ʼ��
  * @param  critical:�ٽ�ֵ�ṹ��ָ��
  *    get:��ǰ��ȡ���ĽǶȣ������ǽǻ��е�Ƕȣ�
  * @retval void
  */
void Critical_Handle_Init(Critical_t *critical, float get)
{
	
	critical->AngleSum = get;//0;
	critical->CurAngle = get;
	critical->LastAngle = get;  
	
	Cloud_Angle_Target[YAW][GYRO] = get; 

}


float speed_threshold = 5.f;//�ٶȹ���
float debug_speed;//�����Ҹ�,һ�㶼��1����,debug��
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//�����ٶ�
		
		
//		if ((S->speed - S->processed_speed) < -speed_threshold)
//		{
//			S->processed_speed = S->processed_speed - speed_threshold;//�ٶ�б�±仯
//		}                                                                                           //DEBUG��
//		else if ((S->speed - S->processed_speed) > speed_threshold)
//		{
//			S->processed_speed = S->processed_speed + speed_threshold;//�ٶ�б�±仯
//		}
		

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//��������ٶ�
}

/**
  * @brief  ����YAWƫ�����ĽǶ�,���̸���ģʽ��
  * @param  void
  * @retval sAngleError,ƫ��Ƕ�ֵ,CAN�����Ļ�е�Ƕ�
  */
float GIMBAL_GetOffsetAngle(void)
{
  float sAngleError = 0;
	
	if((angleMotorYaw < 4469.0f) && (angleMotorYaw > 4369.0f))//0.034.f   4250    3750
	{
		sAngleError = 0;
	}
	else
	{
		#if YAW_POSITION ==	YAW_DOWN
            sAngleError =  Cloud_Angle_Measure[YAW][MECH];
        #else
		    sAngleError =  Cloud_Angle_Measure[YAW][CHANGE];//CHANGE
		#endif
	}
	
	return  sAngleError;
}


///**
//  * @brief  ����YAWƫ�����ĽǶ�,���̸���ģʽ��
//  * @param  void
//  * @retval sAngleError,ƫ��Ƕ�ֵ,CAN�����Ļ�е�Ƕ�
//  */
//float GIMBAL_GetOffsetAngle_Half(void)
//{
//  float sAngleError = 0;
//	
//	if((angleMotorYaw < 4696.0f) && (angleMotorYaw > 4296.0f))//0.034.f   4250    3750
//	{
//		sAngleError = 0;
//	}
//	else
//	{
//		#if YAW_POSITION ==	YAW_DOWN	
//            sAngleError =  Cloud_Angle_Measure[YAW][CHANGE];
//        #else
//		    sAngleError =  Cloud_Angle_Measure[YAW][MECH];//CHANGE
//		#endif
//	}
//	
//	return  sAngleError;
//}

/*---------------------------------------------------------------------------------���飬Ԥ��yaw��ĸ�������------------------------------------------------------------------------------*/
/**
  * @brief  ����yaw��Ԥ���Ƿ��Ѿ�����
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfAuto_MobPre_Yaw(void)
{
return FALSE;
}

/**
  * @brief  yaw�Ὺ��Ԥ���ʱ����̨�Ƿ�λ
  * @param  void
  * @retval TRUE��λ�ɴ�   FALSEû��λ��ֹ��
  * @attention ���Ҹ����ӳ٣�����ʱ�ǵ����㷴��;�ֹʱ���ӳ�
  */
bool GIMBAL_MOBPRE_YAW_FIRE(void)
{
return FALSE;
}

/*---------------------------------------------------------------------------------------���yaw��pitch���Ƿ��ƶ���λ-----------------------------------------------------------------*/


/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------�ڱ��������Ԥ�⸨������-----------------------------------------------------------------*/
/**
  * @brief  �Ƿ��������ڱ�
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
bool GIMBAL_AUTO_PITCH_SB(void)
{
return FALSE;
}


/**
  * @brief  �Ƿ����еȾ��������ڱ�,�Ӵ�Ԥ��
  * @param  void
  * @retval TRUE   FALSE
  * @attention ����̧ͷ�Ƕ�̫������Ϊ�ڴ��ڱ�
  */
float pitch_sb_error = 0;
bool GIMBAL_AUTO_PITCH_SB_SK(void)
{
return FALSE;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------�Ƿ����������ģʽ---------------------------------------------------------------------*/
/**
  * @brief  �Ƿ�������ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_If_Base(void)
{
    if (actGimbal == GIMBAL_BASE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------�Ƿ������Ƿ��ִ��Ƿ������Ƿ�С��---------------------------------------------------------------*/
/**
  * @brief  �Ƿ������ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfBuffHit(void)
{
 if (actGimbal == GIMBAL_BUFF || actGimbal == GIMBAL_SM_BUFF|| actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �Ƿ����ֶ����ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_IfManulHit(void)
{
    if (actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �Ƿ�������ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_If_Big_Buff(void)
{
    if (actGimbal == GIMBAL_BUFF)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �Ƿ�����С��ģʽ
  * @param  void
  * @retval TRUE����  FALSE�ر�
  * @attention 
  */
bool GIMBAL_If_Small_Buff(void)
{
    if (actGimbal == GIMBAL_SM_BUFF)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  ���yaw�Ƿ��ƶ���λ
  * @param  void
  * @retval TRUE��λ�ɴ�   FALSEû��λ��ֹ��
  * @attention 
  */

bool GIMBAL_BUFF_YAW_READY(void)
{
return FALSE;
}

/**
  * @brief  ���pitch�Ƿ��ƶ���λ
  * @param  void
  * @retval TRUE��λ�ɴ�   FALSEû��λ��ֹ��
  * @attention 
  */

bool GIMBAL_BUFF_PITCH_READY(void)
{
 return FALSE;
}
/*---------------------------------------------------���½ǶȵĻ�е�ǶȺ������ǽǶȵĺ���------------------------------------------------------------*/

//���������̨��ֵ�ĽǶ�
static fp32 motor_ecd_to_angle_change(uint16_t ecd)
{
    int32_t relative_ecd = ecd - 4396;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad ;
}


//���������̨��ֵ�ĽǶ�
static fp32 motor_ecd_to_angle_change1(uint16_t ecd)
{
    int32_t relative_ecd = ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad ;
}


fp32 Angle_Measure[2][3];
/**
  * @brief  ������̨��е�Ƕ�,���ٶ�,����ֵ,can1�ж��е���
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_UpdateAngle( char ID, int16_t angle )
{
if (ID == PITCH)
	{
		angleMotorPit = angle;
		Cloud_Angle_Measure[PITCH][MECH]  = motor_ecd_to_angle_change(angleMotorPit);	
	}
	else if (ID == YAW)
	{
		angleMotorYaw = angle;
		Cloud_Angle_Measure[YAW][MECH]  = motor_ecd_to_angle_change(angleMotorYaw);
		Angle_Measure[YAW][MECH] = angleMotorYaw;
		Cloud_Angle_Measure[YAW][CHANGE]  = motor_ecd_to_angle_change(angleMotorYaw);
	}
}

void GIMBAL_UpdateSpeed( char ID, int16_t speed )
{
	if (ID == PITCH)
	{
		speedMotorPit = speed;
		Cloud_Speed_Measure[PITCH][MECH]  = speedMotorPit;
	}
	else if (ID == YAW)
	{
		speedMotorYaw = speed;
		Cloud_Speed_Measure[YAW][MECH]  = speedMotorYaw;
	}
}

void GIMBAL_UpdateCurrent( char ID, int16_t current )
{
	if (ID == PITCH)
	{
		currentMotorPit = current;
		Cloud_Current_Measure[PITCH][MECH]  = currentMotorPit;
	}
	else if (ID == YAW)
	{
		currentMotorYaw = current;
		Angle_Measure[YAW][MECH] = angleMotorYaw;
		Cloud_Current_Measure[YAW][MECH]  = currentMotorYaw;
	}
}

/**
  * @brief  ������̨��̬,500HZ,loop�е���
  * @param  void
  * @retval void
  * @attention �Ƕ��ʶȷŴ�
  */
float AngleMpuYaw[2];
float AngleMpuPitch[2];
float AngleMpuRoll[2];

void GIMBAL_MPU_Update(void)
{
	//��ȡ������  �Ƕ�   ���ٶ�   
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	
	AngleMpuYaw[NOW]   = angleMpuYaw-AngleMpuYaw[LAST];			 //22
	AngleMpuPitch[NOW] = angleMpuPitch-AngleMpuPitch[LAST];  //-166
	AngleMpuRoll[NOW]  = angleMpuRoll-AngleMpuRoll[LAST]; 	 //4.2
	
			//�������ǽǶȷŴ�,���Ŵ���������ģʽ�ڻ�PҪ���ܴ�,�᲻�õ�
		Cloud_Angle_Measure[PITCH][GYRO]  =  (AngleMpuPitch[NOW]*PI)/180;
	  Cloud_Angle_Measure[YAW][GYRO] = (AngleMpuYaw[NOW]*PI)/180 ;
	  //theta_format(Cloud_Angle_Measure[YAW][GYRO]);
	
		//���ٶȸ���
		Cloud_Palstance_Measure[PITCH][MECH] = ((palstanceMpuPitch + PALST_COMPS_PITCH)*PI)/180;
		Cloud_Palstance_Measure[YAW][MECH]   = ((palstanceMpuYaw+PALST_COMPS_YAW)*PI)/180;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH)/10;   //ֱ�Ӷ������ǽ��ٶ�
		Cloud_Palstance_Measure[YAW][GYRO]   = (palstanceMpuYaw+PALST_COMPS_YAW)/10;  //��������ó��Ľ��ٶ�
}


void MPU_Update_last(void)
{
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	AngleMpuYaw[LAST] = angleMpuYaw;
	AngleMpuPitch[LAST] = angleMpuPitch;
	AngleMpuRoll[LAST] = angleMpuRoll;
}

float Error[2][2];
void Gimbal_Error_Read(void)
{
	Error[YAW][MECH]= Cloud_Angle_Target[YAW][MECH] -Cloud_Angle_Measure[YAW][MECH];
	Error[YAW][GYRO]= Cloud_Angle_Target[YAW][GYRO] -Cloud_Angle_Measure[YAW][GYRO];
	Error[PITCH][MECH]= Cloud_Angle_Target[PITCH][MECH] -Cloud_Angle_Measure[PITCH][MECH];
	Error[PITCH][GYRO]= Cloud_Angle_Target[PITCH][GYRO] -Cloud_Angle_Measure[PITCH][GYRO];
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;

}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//pid��������
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

#define TOP_STEP0    0		//ʧ�ܱ�־
#define TOP_STEP1    1		//SW1��λ��־
#define TOP_STEP2    2		//���ֿ��ر�־
uint8_t	TOP_Switch = 0;//����ң��ģʽ���ر�־λת��
bool TOP_Rc_Switch(void)
{
	if (IF_RC_SW2_DOWN)//��еģʽ
	{
			if (TOP_Switch == TOP_STEP1)
			{
				TOP_Switch = TOP_STEP2;
			}
			 else if (TOP_Switch == TOP_STEP2)
			{
				TOP_Switch = TOP_STEP0;
			}
	}
	else
	{
		TOP_Switch = TOP_STEP1;
	}
	
	if (TOP_Switch == TOP_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	
}

