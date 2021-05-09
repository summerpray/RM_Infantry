/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
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

#ifndef GIMBALTASK_H
#define GIMBALTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "remote_control.h"


//�����ʼ�� ����һ��ʱ��
#define GIMBAL_TASK_INIT_TIME 201

#define EEE   99999999999     //�������뾯���õ�

#define PALST_COMPS_YAW        (-13)     //������YAW���ٶȲ���
#define PALST_COMPS_PITCH      (105)      //������PITCH���ٶȲ���

/*----------------------------------------�ٶȻ�--------------------------------------------*/
//pitch �ٶȻ� PID�����Լ� PID���������������
#define PITCH_SPEED_PID_KP 100.0f
#define PITCH_SPEED_PID_KI 0.0f
#define PITCH_SPEED_PID_KD 0.0f
#define PITCH_SPEED_PID_MAX_OUT 10000.0f
#define PITCH_SPEED_PID_MAX_IOUT 1000.0f

//yaw �ٶȻ� PID�����Լ� PID���������������
#define YAW_SPEED_PID_KP 25.0f//25
#define YAW_SPEED_PID_KI 0.00f
#define YAW_SPEED_PID_KD 0.0f
#define YAW_SPEED_PID_MAX_OUT 10000.0f//10000
#define YAW_SPEED_PID_MAX_IOUT 500.0f//500

/*----------------------------------------�ǶȻ�--------------------------------------------*/
//**������ģʽ**//
//pitch �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������                        
#define PITCH_GYRO_ABSOLUTE_PID_KP 800.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 1.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 3.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10000.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 100.0f

//yaw �ǶȻ� �Ƕ��������ǽ��� PID�����Լ� PID���������������
#define YAW_GYRO_ABSOLUTE_PID_KP 3000.0f//3500
#define YAW_GYRO_ABSOLUTE_PID_KI 0.1f
#define YAW_GYRO_ABSOLUTE_PID_KD 4.0f

#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT 10000.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT 100.0f

//**����ģʽ**//

//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_KEY_PID_KP 1000.0f//1950
#define PITCH_KEY_PID_KI 0.1f
#define PITCH_KEY_PID_KD 1.0f

#define PITCH_KEY_PID_MAX_OUT 10000.0f
#define PITCH_KEY_PID_MAX_IOUT 100.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_KEY_PID_KP 2000.0f//1950
#define YAW_KEY_PID_KI 0.3f
#define YAW_KEY_PID_KD 3.0f

#define YAW_KEY_PID_MAX_OUT 10000.0f
#define YAW_KEY_PID_MAX_IOUT 100.0f


//**��еģʽ**//
//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define PITCH_ENCODE_RELATIVE_PID_KP 800.0f//1950
#define PITCH_ENCODE_RELATIVE_PID_KI 3.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10000.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 100.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������
#define YAW_ENCODE_RELATIVE_PID_KP 3000.0f//5000
#define YAW_ENCODE_RELATIVE_PID_KI 0.1f
#define YAW_ENCODE_RELATIVE_PID_KD 3.0f

#define YAW_ENCODE_RELATIVE_PID_MAX_OUT 10000.0f//10000
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT 100.0f//100

#define max_yaw_relative_angle     (PI/6)
#define min_yaw_relative_angle       -(PI/6)
#define max_pitch_relative_angle   (PI/8)
#define min_pitch_relative_angle  -(PI)
#define mid_yaw_angle                0       //��װʱ���� �����������
#define mid_pitch_angle          -(PI/1.5)

//��̨����pid
#define  GIMBAL_FOLLOW_CHASSIS_PID_KP 0.10f  //0.15
#define  GIMBAL_FOLLOW_CHASSIS_PID_KI 0.001f
#define  GIMBAL_FOLLOW_CHASSIS_PID_KD 0.01f
#define  GIMBAL_FOLLOW_CHASSIS_PID_MAX_OUT 3.0f //3.0
#define  GIMBAL_FOLLOW_CHASSIS_PID_MAX_IOUT 0.2f

//pitch �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID��������������� ����PID
#define PITCH_AUTO_PID_KP 1000.0f//1950
#define PITCH_AUTO_PID_KI 0.1f
#define PITCH_AUTO_PID_KD 1.0f
#define PITCH_AUTO_PID_MAX_OUT 10000.0f
#define PITCH_AUTO_PID_MAX_IOUT 100.0f

//yaw �ǶȻ� �Ƕ��ɱ����� PID�����Լ� PID���������������  ����PID
#define YAW_AUTO_PID_KP 2000.0f//1950
#define YAW_AUTO_PID_KI 0.5f
#define YAW_AUTO_PID_KD 3.0f
#define YAW_AUTO_PID_MAX_OUT 10000.0f
#define YAW_AUTO_PID_MAX_IOUT 100.0f

//���׿������˲�����λ��
#define KF_ANGLE	0
#define KF_SPEED	1

//��ͷ��̨�ٶ�
#define TurnSpeed 0.04f

//ң����������������Ϊң�������ڲ��죬ҡ�����м䣬��ֵ��һ��Ϊ��
#define RC_deadband 10


//yaw��pitch�Ƕ���ң�����������
#define Yaw_RC_SEN -0.000006f//0.000011
#define Pitch_RC_SEN -0.000006f //0.005


//yaw,pitch�ǶȺ��������ı���
#define Yaw_Mouse_Sen -0.0001f
#define Pitch_Mouse_Sen -0.0002f
#define Yaw_Mouse_ramp 0.05f
#define Pitch_Mouse_ramp 0.05f


//��̨����������ʱ��ʹ�õı���
#define Yaw_Encoder_Sen 0.01f
#define Pitch_Encoder_Sen 0.01f


//��̨��������
#define GIMBAL_CONTROL_TIME 1


//�������ֵ����Լ���ֵ
#define Half_ecd_range 4396   //4446
#define ecd_range 8191


//�������ֵת���ɽǶ�ֵ
#ifndef Motor_Ecd_to_Rad
#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192 0.000766990394f
#endif

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //���ԭʼֵ����
    GIMBAL_MOTOR_GYRO,    //��������ǽǶȿ���
    GIMBAL_MOTOR_ENCONDE, //�������ֵ�Ƕȿ���
} gimbal_motor_mode_e;

typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Gimbal_PID_t;



void GIMBAL_task(void *pvParameters);
/*----------------------------------myself---------------------------------------*/

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);   //��̨PID��ʼ�������ڽǶȻ�
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);//��̨PID���㣬���ڽǶȻ�
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);//��̨PID����


void RC_Set_Mode(void);  //ң����ѡ�����ģʽ
void KEY_Set_Mode(void); //����ѡ�����ģʽ

void GIMBAL_Set_Control(void); //ң����ģʽ������̨��Ŀ��ֵ
void GIMBAL_Set_Key_Control(void); //����ģʽ������̨��Ŀ��ֵ

void GIMBAL_Behaviour_Control_Set(fp32 add_yaw_mech, fp32 add_yaw_gyro, fp32 add_pitch_mech , fp32 add_pitch_gyro);//��̨���ݲ�ͬ״̬���ò�ͬ�ĺ���
void gimbal_absolute_angle_control(fp32 yaw, fp32 pitch);//��̨������ģʽ�Ŀ��ƺ�������Ҫд��180���ͷ

void GIMBAL_absolute_angle_limit(fp32 add_yaw_gyro_angle, fp32 add_pitch_gyro_angle);//������ģʽң��������
	
#define YAW 0
#define PITCH 1

#define MECH 0
#define GYRO 1
#define TOP 2
#define CHANGE 3

#define NOW  0
#define LAST 1

//��̨ģʽѡ��
typedef enum
{
	CLOUD_MECH_MODE = 0,
	CLOUD_GYRO_MODE = 1,
	CLOUD_KEY_MODE = 2,
	CLOUD_TOP_MODE =3,
} GimbalCtrlMode;



/* ��̨����ģʽ:
   
   ��ͨ             	NORMAL
   ��ͷ180��             AROUND
   ���             	BUFF
   ����,pitchˮƽ   	LEVEL
   ��еģʽpitcḩͷ	HIGH
   ����Ťͷ90��          TURN
*/
typedef enum
{
	GIMBAL_NORMAL  = 0,//����ģʽ,����ģʽѡ��
	GIMBAL_AROUND  = 1,//180���ͷ
	GIMBAL_BUFF    = 2,//���ģʽ,��
	GIMBAL_LEVEL   = 3,//���ֿ���,��̨ˮƽ
	GIMBAL_MANUAL  = 4,//�ֶ����ģʽ
	GIMBAL_SM_BUFF = 5,//С��
	GIMBAL_TURN    = 7,//90��Ťͷ
	GIMBAL_AUTO    = 8,//����
	GIMBAL_BASE    = 9,//��ͷ�������
	
}eGimbalAction;



typedef struct  //�Ӿ�Ŀ���ٶȲ���
{
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
}speed_calc_data_t;

/*       �ٽ�ֵ����ṹ��        */
typedef struct 
{

	float LastAngle;       //��һ�ζ�ȡ�ĽǶ�
	float CurAngle;	       //��ǰ��ȡ�ĽǶ�
	float AngleSum;	       //�Ƕ��ۼ�ֵ
	
}Critical_t;

void GIMBAL_InitCtrl(void);
void GIMBAL_Rc_Ctrl( void ); //ң����ģʽ
void GIMBAL_Key_Ctrl(void); //����ģʽ
void vPitch_Mech_PositionLoop(void);  //error_delta �����ǽ��ٶ�
void vPitch_Gyro_PositionLoop(void);
void vYaw_Gyro_PositionLoop(void);
void vYaw_Mech_PositionLoop(void);

void GIMBAL_PositionLoop(void);
void GIMBAL_PositionLoop_AUTO(void);
void GIMBAL_kPID_Init(void);
void GIMBAL_CanSend(void);
void GIMBAL_UpdateCurrent( char ID, int16_t current );
void GIMBAL_UpdateSpeed( char ID, int16_t speed );
void GIMBAL_UpdateAngle( char ID, int16_t angle );
void GIMBAL_MPU_Update(void);
void MPU_Update_last(void);
void Gimbal_Error_Read(void);

void Critical_Handle_Init(Critical_t *critical, float get);
float Gimbal_Yaw_Gryo_AngleSum(Critical_t *critical, float get);
void Gimbal_Chass_Separ_Limit(void);
float GIMBAL_GetOffsetAngle(void);
float GIMBAL_GetOffsetAngle_Half(void);
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);

void GIMBAL_NORMAL_Mode_Ctrl(void);//��̨������Ӧģʽ
void GIMBAL_LEVEL_Mode_Ctrl(void);//����ģʽ
void GIMBAL_AUTO_Mode_Ctrl(void);//����ģʽ
void GIMBAL_AUTO_PREDICT_Mode_Ctrl(void);//����Ԥ��ģʽ
void	GIMBAL_BUFF_Mode_Ctrl_Gimbal(void);//��̨���������ͷ����̨
void GIMBAL_BASE_Mode_Ctrl(void);///��ͷ����ģʽ
bool AUTOMode(void);//�����Ƿ�ʶ��Ŀ��

bool GIMBAL_IfBuffHit(void); //�Ƿ������ģʽ
bool GIMBAL_IfManulHit(void); //�Ƿ����ֶ����ģʽ
bool GIMBAL_IfAuto_MobPre_Yaw(void);//����yaw��Ԥ���Ƿ��Ѿ�����
bool GIMBAL_MOBPRE_YAW_FIRE(void);//yaw�Ὺ��Ԥ���ʱ����̨�Ƿ�λ
bool GIMBAL_BUFF_YAW_READY(void);//���YAW���Ƿ��ƶ���λ
bool GIMBAL_BUFF_PITCH_READY(void);//���PITCH�Ƿ��ƶ���λ
bool GIMBAL_AUTO_PITCH_SB(void);//�Ƿ��������ڱ�
bool GIMBAL_AUTO_PITCH_SB_SK(void);//�Ƿ����еȾ��������ڱ�
bool GIMBAL_If_Base(void);//�Ƿ��ڵ������
bool GIMBAL_If_Big_Buff(void); //�Ƿ�������
bool GIMBAL_If_Small_Buff(void);//�Ƿ�����С��
bool TOP_Rc_Switch(void);
float GIMBAL_MPU_angle(float *get);//���������ǲ���ֵ


void moni_chassis_action(void);//�����õ�
#endif
