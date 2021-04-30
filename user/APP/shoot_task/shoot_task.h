#ifndef SHOOT_TASK__H
#define SHOOT_TASK__H

#include "main.h"
#include  <math.h>
#include <stdlib.h>

#define 	REVOL_SPEED_GRID      8			  //���̸���
#define   AN_BULLET         (24576.0f)		//�����ӵ����λ������ֵ
#define   REVOL_SPEED_RATIO   2160        //�����һ��תһȦ,2160ת��ת��,60*36,����Ƶ�ٳ��Բ��̸����Ϳɵ���Ӧ��Ƶ�µ�ת��


/* �������ͷԤ���� */
#define    BUFF_CAM_CHAS	0
#define    BUFF_CAM_GIMB	1
#define    BUFF_CAM_TYPE	BUFF_CAM_GIMB

/******����,�����߼�����̨����*********/

//���̵��ģʽ,λ�û����ٶȻ�
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//����ģʽѡ��
//���̵��ģʽ,λ�û����ٶȻ�
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;


//ң��ģʽ�µ�һЩ��־λ
#define    FRIC_STEP0    0
#define    FRIC_STEP1    1
#define    FRIC_STEP2    2


//�ٶ�ѡ��
#define FRI_OFF  	0
#define FRI_LOW  	1		//����
#define FRI_MID  	2		//����	
#define FRI_HIGH 	3		//����
#define FRI_MAD  	4		//�������
#define FRI_SENTRY  5		//�ڱ�����


typedef enum
{
	SHOOT_NORMAL       =  0,//���ģʽѡ��,Ĭ�ϲ���
	SHOOT_SINGLE       =  1,//����
	SHOOT_TRIPLE       =  2,//������
	SHOOT_HIGHTF_LOWS  =  3,//����Ƶ������
	SHOOT_MIDF_HIGHTS  =  4,//����Ƶ������
	SHOOT_BUFF         =  5,//���ģʽ
	SHOOT_AUTO         =  6,//�����Զ����
}eShootAction;

//Ħ���ֵ��ID
typedef enum
{
	Fric_Left =  0,  
	Fric_Right = 1,  
	
}Fric_Motor_t;

void shoot_task(void *pvParameters); //������
void REVOLVER_Rest(void);    //��������
void SHOOT_InitArgument(void);  //���̲�����ʼ��
void REVOLVER_Rc_Ctrl(void);  //ң��������
void REVOLVER_Key_Ctrl(void);  //���̿���
bool REVOLVER_Rc_Switch(void); //����ң�ش�
void Revolver_Angle_Rest(void); //���̽Ƕ�����
void REVOL_PositStuck(void);//λ�û���������
void REVOL_SpeedStuck(void);//�ٶȻ���������
void REVOL_PositionLoop(void); //λ�û�PID
void REVOL_SpeedLoop(void); //�ٶȻ�PID
void Fric_SpeedLoop(Fric_Motor_t Fric_Motor);//Ħ�����ٶȻ�PID
void Fric_Speed_PID_Calculate(void);//Ħ�����ٶȻ�PID����
void REVOL_UpdateMotorAngleSum(void);//ͳ��ת���Ƕ��ܺ�
void REVOLVER_UpdateMotorSpeed(int16_t speed);//�ж϶�ת��
void REVOLVER_UpdateMotorAngle(int16_t angle);//�ж϶��Ƕ�
void Reset_Fric(void);//��λĦ����
void ReFric(void);//����Ħ�������ǹ��
void Fric_Power_Change(void);//�ı�Ħ�����ٶ�
void SHOOT_CANbusCtrlMotor(void);//can���ͺ���


void Fric_Key_Ctrl(void); //����ģʽĦ���ֵĿ���
void SHOOT_NORMAL_Ctrl(void);  //����ģʽ�µ���ͨģʽ
void SHOOT_SINGLE_Ctrl(void); //�������ģʽ
void SHOOT_TRIPLE_Ctrl(void); //��������
void SHOOT_HIGHTF_LOWS_Ctrl(void);//����Ƶ�����ٿ���
void SHOOT_MIDF_HIGHTS_Ctrl(void);//����Ƶ�����ٿ���
void SHOOT_AUTO_Ctrl(void);//�����������
void SHOOT_BUFF_Ctrl(void);//����������,����������ͷλ�ڵ���
void SHOOT_BUFF_Ctrl_Gimbal(void);//���������ƣ�����ͷ����̨
void REVOLVER_KeyPosiCtrl(void);//����ģʽ����λ�û�����
void REVOLVER_KeySpeedCtrl(void);//����ģʽ�����ٶȻ�����
void Fric_RC_Ctrl(void);
void Set_Fric_Speed(int8_t speed_mode); //����Ħ���ֵ�λ

void REVOLVER_SHAKE(void);//����ƽ̨��
bool FRIC_RcSwitch(void);
void Friction_Ramp(void);
void Fric_mode(uint16_t speed);
#endif 
