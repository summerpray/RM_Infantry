/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
  * @note       
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
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"
//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357



extern void chassis_task(void *pvParameters);


//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 10000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.01f
#define M3505_MOTOR_SPEED_PID_KD 3.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define M3505_MOTOR_SPEED_KEY_PID_KP 5000.0f//6000
#define M3505_MOTOR_SPEED_KEY_PID_KI 0.0f
#define M3505_MOTOR_SPEED_KEY_PID_KD 0.0f
#define M3505_MOTOR_SPEED_KEY_PID_MAX_OUT 10000.0f
#define M3505_MOTOR_SPEED_KEY_PID_MAX_IOUT 1000.0f

//������ת����PID ����ģʽ
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_KP 0.3f  //0.15
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_OUT 3.0f //3.0
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_IOUT 0.1f

//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.15f  //0.15
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.01f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 3.0f //3.0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.1f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f    //һ�׵�ͨ�˲�����
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002

//����ģʽ���̹����л�����
#define LOW 0
#define HIGH 1

//���̵������ٶ�
#define MAX_WHEEL_SPEED 2.0f  

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.0f
//�����˶����������ת�ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Z 0.05f


//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.004f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.004f

#define CHASSIS_RC_DEADLINE 10


//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.5f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.001f


//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

void chassis_feedback_update(void);   //������ݸ���
/*--------------------myself-------------*/
/*----------------------------------------------------------������ת�ٵ�����-----------------------------------------------------------------------------------------*/

#define 	  Omni_Speed_Max              9000     //����ˮƽ�ƶ��ٶ��޷�,��ֹ����ģʽ���ٶȳ������ֵ
#define		  STANDARD_MAX_NORMAL         9000     //ƽ�ؿ�������ٶȣ���ֹҡ�˱���*660�������ֵ
#define     Omni_SupCap_Max             10000    //��סshift����ʱ����ٶȣ����������ٸ���

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------����Ťͷ����ز���--------------------------------------------------------------------------------------*/

#define     theta_error                 0
#define		  REVOLVE_MAX_NORMAL          9000     //ƽ��Ťͷ����ٶ�
#define     REVOLVE_KD                 (350.f)
#define     REVOLVE_ANGLE               35
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------��ͬģʽ��,б�º�����Ӧ��ʱ��ֵ,һ������ͨ�ٶȾ���-----------------------------------------------------------------*/

#define     TIME_INC_NORMAL             10	       //����б��,Խ�������ٶ�Խ��,���ʱ��Խ��
#define     TIME_DEC_NORMAL             500        //����б��,Խ���С�ٶ�Խ��(һ��Ҫ��INC��һ��,�����ɿ������ܸ���Ϊ0,̫�������ɵ���ͣ������ʱ����Ծ)

#define     TIME_INC_SLOW               1		       //����ģʽ���ٶȱ仯����
#define     TIME_INC_UPUP               3		       //�ֶ�����ģʽ���ٶȱ仯����

#define     TIME_INC_SALTATION          1          //ͻȻ����������ٶȱ仯����

#define     REVOLVE_SLOPE_NORMAL        80         //������ͨģʽб��,Խ��Խ��,���ʱ��Խ��
#define     REVOLVE_SLOPE_CORGI         50         //����Ťƨ��ģʽб��,Խ��Խ��,���ʱ��Խ��
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define     LIMIT_CHASSIS_MAX        9000       //������������µ��̵������������
  
/*----------------------------------------------��ͬģʽ�µ�����ٶ�����--------------------------------------------------------------------------------------*/

#define    REVOLVE_MAX_CORGI         9000       //����Ťƨ������ٶ�,̫����ýǶȹ���

#define    STANDARD_MAX_SLOW         1500	      //����ģʽ��ˮƽ�ƶ��ٶ�
#define    REVOLVE_MAX_SLOW          2000       //����ģʽ��Ťͷ�ٶ�
 
#define    STANDARD_MAX_UPUP       3000       //�ֶ�����ģʽ��ˮƽ�ƶ��ٶ�
#define    REVOLVE_MAX_UPUP        9000       //�ֶ�����ģʽ��Ťͷ�ٶ�

#define    LIMIT_CHASSIS_MAX         9000       //������������µ��̵������������
#define    CHAS_CURRENT_LIMIT        36000     //�ĸ����ӵ��ٶ��ܺ����ֵ,�������*4,�޹��ʵ���������

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//���̵����޷�
#define     iTermChassis_Max            3000     //΢���޷�

/*----------------------------------------------���̹����ز����޸�--------------------------------------------------------------------------------------*/

#define RADIUS     76      //���ְ뾶
#define PERIMETER  478     //�����ܳ�
#define WHEELTRACK 512     //�����־�
#define WHEELBASE  378     //ǰ�����

#define GIMBAL_Y_OFFSET 0 //��̨����30

#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //���������

#define RADIAN_COEF 57.3f  
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/
typedef enum
{
	
	CHASSIS_MECH_MODE = 0,//��е
	CHASSIS_GYRO_MODE = 1,//������,���̸�����̨
	CHASSIS_SHAKE_MODE = 2, //ҡ��ģʽ
	CHASSIS_TOP_MODE =3,
	
} ChassisCtrlMode;

typedef enum	
{
	CHASSIS_NORMAL   = 0,//��ͨģʽ������ǰ��
	CHASSIS_SHAKE    = 1,//Ťƨ��ģʽ
	CHASSIS_REVOLVE  = 2,//С����ģʽ
	CHASSIS_UP       = 3,//����ģʽ
	CHASSIS_MISS     = 4,//�Զ�����ģʽ
	CHASSIS_SLOW     = 5, //���ײ���ģʽ
	CHASSIS_BUFF     = 6,//���ģʽ������Ϊ��еģʽ
 } ChassisActionMode;

//���̵��ID
typedef enum
{
	
	LEFT_FRON_201 = 0,  // ��ǰ
	RIGH_FRON_202 = 1,  // ��ǰ
	LEFT_BACK_203 = 2,  // ���
	RIGH_BACK_204 = 3,  // �Һ�
	
} ChassisWheel;


void Chassis_Motor_Speed_PID(void) ;
void Chassis_Motor_Key_PID( ChassisWheel Wheel ) ; //����ģʽʱ���PID
void Chassis_MotorOutput(void);
void CHASSIS_REST(void);
void Chassis_Set_Mode(void); //ң��������ģʽ
void Chassis_Rc_Control(void);//ң������������Ϊ Chassis_Move_X  Chassis_Move_Y
void Chassis_Set_Contorl(void);//��ͬģʽ��ͬ����ʽ
void Chassis_Set_key_Contorl(void); //����ģʽ��ͬģʽ��ͬ����
void Chassis_Omni_Move_Calculate(void);
void Chassis_RC_Ctrl(void);
void Chassis_Key_Ctrl(void);
void Chassis_Power_Change(void); //����
void Chassis_NORMAL_Mode_Ctrl(void);
void CHASSIS_CANSend(void);
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle );
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed );
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current );
void Chassis_Init(void);
void Angle_error(void);
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp);
void Chassis_Motor_Speed_PID_KEY(void);
float Chassis_Z_Corgi(float get, float set);
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_UPUP_Mode_Ctrl(void);
void CHASSIS_MISS_Mode_Ctrl(void);
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp );
void Chassis_Keyboard_Move_TOP_Calculate(int16_t sMoveMax, int16_t sMoveRamp);
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax );
bool CHASSIS_IfActiveMode(void);//��ȡ�����ƶ�ģʽ
bool Chassis_IfSHAKE(void);//�Ƿ���Ťƨ��

#endif
