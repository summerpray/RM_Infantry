/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       vision.c/h
  * @brief      ����Ӿ����ݶ�ȡ�봫�䡣
  * @note       ��Ҫ���Ӿ����ݶ�ȡ���Ӿ����ݴ��䣬��Ҫ�������飬����������ָ�����һЩ
	              �Ӿ��ĸ���������
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     2019-11-16       XYS              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include <vision.h>
#include "math.h"
#include "remote_control.h"
#include "crc.h"
#include "uart7.h"
#include "gimbal_task.h"
#include "kalman.h"
#include "chassis_task.h"
#include "judge.h"
#include "FreeRTOS.h"
#include "task.h"
#include "delay.h"

extern RC_ctrl_t rc_ctrl;

//�Ƕȳ�ʼ������
float Vision_Comps_Yaw = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;			 //�̶���������С�����Ӱ��
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST; //���ݾ��벹��

VisionSendHeader_t VisionSendHeader; //֡ͷ

VisionActData_t VisionActData; //�ж�ģʽ�ṹ��

VisionRecvData_t VisionRecvData; //�������ݽṹ��

VisionSendData_t VisionSendData; //�������ݽṹ��

uint8_t Attack_Color_Choose = ATTACK_NONE; //Ĭ�ϲ�ʶ��

//����Ƿ�װ����
uint8_t Vision_Armor = FALSE;

//�Ӿ��Ƿ���������,FALSEû��,TRUE�����µ�
uint8_t Vision_Get_New_Data = FALSE;

//�ǶȲ���,���͸��Ӿ�
float Vision_Comps_Yaw_Send = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;
float SB_K_comps = 3.f;

/**
  * @brief  ��ȡ�Ӿ���Ϣ
  * @param  uart7��������
  * @retval void
  * @attention  IRQִ��
  */
uint8_t Vision_Time_Test[2] = {0}; //��ǰ���ݺ���һ������
uint8_t Vision_Ping = 0;		   //����ʱ����
void Vision_Read_Data(uint8_t *ReadFormUart7)
{
	//�ж�֡ͷ�����Ƿ�Ϊ0xA5
	if (ReadFormUart7[0] == VISION_SOF)
	{
		//֡ͷCRC8У��
		if (Verify_CRC8_Check_Sum(ReadFormUart7, VISION_LEN_HEADER) == TRUE)
		{
			//֡βCRC16У��
			if (Verify_CRC16_Check_Sum(ReadFormUart7, VISION_LEN_PACKED) == TRUE)
			{
				//�������ݿ���
				memcpy(&VisionRecvData, ReadFormUart7, VISION_LEN_PACKED);
				Vision_Get_New_Data = TRUE; //����Ӿ����ݸ�����

				//֡����
				Vision_Time_Test[NOW] = xTaskGetTickCount();
				Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST]; //����ʱ����
				Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
				if (GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE) //��Ǵ�������ˣ���װ����
				{
					if (VisionRecvData.identify_buff == 2) //��2˵����װ�װ���
					{
						Vision_Armor = TRUE; //��2��װ��
					}
				}
			}
		}
	}
}

/**
  * @brief  �����Ӿ�ָ��
  * @param  CmdID
  * @retval void
  * @attention  ��Э���������ݷ���
  *				CmdID   0x00   �ر��Ӿ�
  *				CmdID   0x01   ʶ���ɫװ��
  *				CmdID   0x02   ʶ����ɫװ��
  *				CmdID   0x03   С��
  *				CmdID   0x04   ���
  */
uint8_t vision_send_pack[50] = {0}; //����22����
void Vision_Send_Data(uint8_t CmdID)
{
	int i; //ѭ�����ʹ���

	VisionSendHeader.SOF = VISION_SOF;
	VisionSendHeader.CmdID = CmdID; //���Ӿ���˵����Ҫ������

	//д��֡ͷ
	memcpy(vision_send_pack, &VisionSendHeader, VISION_LEN_HEADER);

	//֡ͷCRC8У��Э��
	Append_CRC8_Check_Sum(vision_send_pack, VISION_LEN_HEADER);

	//�м����ݲ��ù�,�Ӿ��ò���,�õ���Ҳ�Ǻ��������Զ�����,�õ��ǶȲ�������
	VisionSendData.pitch_angle = 0.f;
	VisionSendData.yaw_angle = 0.f;
	VisionSendData.distance = 999.99f;
	if (GIMBAL_AUTO_PITCH_SB() == TRUE)
	{
		VisionSendData.lock_sentry = 1; //ʶ���ڱ�����1
	}
	else
	{
		VisionSendData.lock_sentry = 0; //����ʶ���ڱ�����0
	}

	if (GIMBAL_If_Base() == TRUE)
	{
		VisionSendData.base = 1; //������أ���1
	}
	else
	{
		VisionSendData.base = 0; //���ڵ��䣬��0
	}

	VisionSendData.blank_a = 0;
	VisionSendData.blank_b = 0;
	VisionSendData.blank_c = 0;
	memcpy(vision_send_pack + VISION_LEN_HEADER, &VisionSendData, VISION_LEN_DATA);

	//֡βCRC16У��Э��
	Append_CRC16_Check_Sum(vision_send_pack, VISION_LEN_PACKED);

	//������õ�����ͨ��������λ���͵�����ϵͳ
	for (i = 0; i < VISION_LEN_PACKED; i++)
	{
		Uart7_SendChar(vision_send_pack[i]);
	}

	memset(vision_send_pack, 0, 50);
}

/**********************************�Ӿ�����*****************************************/
/**
  * @brief  �Ӿ��ܿ���,ָ�����
  * @param  void
  * @retval void
  * @attention  8λ,ֻ�м���ģʽ���Ӿ�
  */
void Vision_Ctrl(void)
{
	if (1) ////����ģʽ
	{
		if (GIMBAL_IfBuffHit() == TRUE && GIMBAL_IfManulHit() == FALSE) //�Զ����ģʽ
		{
			VisionActData = VISION_BUFF;
		}
		else if (GIMBAL_IfManulHit() == TRUE) //�ֶ�ģʽ
		{
			VisionActData = VISION_MANU;
		}
		else //Ĭ�����ѿ���,�����㻹��˵�㲻���书
		{
			VisionActData = VISION_AUTO;
		}

		switch (VisionActData)
		{
		/*- ��� -*/
		case VISION_BUFF:
			Vision_Buff_Ctrl();
			break;

		/*- ���� -*/
		case VISION_AUTO:
			Vision_Auto_Attack_Ctrl();
			break;

		/*- �ֶ� -*/
		case VISION_MANU:
			Vision_Auto_Attack_Off();
			break;
		}
	}
	else
	{
		Vision_Auto_Attack_Off();
	}
}

/**
  * @brief  �������
  * @param  void
  * @retval void
  * @attention  
  */
#define buff_blue 0
#define buff_red 1
bool Buff_Color = 0;
uint8_t Buff_Type = 0; //����ϵͳָʾ�Ʊ��
void Vision_Buff_Ctrl(void)
{
	/* �����ɫ���ݲ���ϵͳ����ȷ������������ɫ��ͬ */
	//s1��˳ʱ��,����ʱ��,�м䲻ʶ��
	Buff_Color = is_red_or_blue();
	if (GIMBAL_If_Small_Buff() == TRUE) //С��ģʽ
	{
		if (Buff_Color == buff_blue)
		{
			Vision_Send_Data(VISION_BBUFF_STAND); //��С
			Buff_Type = VISION_BBUFF_ANTI;		  //ָʾ��Ϊ��ɫ
		}
		else if (Buff_Color == buff_red)
		{
			Vision_Send_Data(VISION_RBUFF_STAND); //��С
			Buff_Type = VISION_RBUFF_CLOCKWISE;	  //ָʾ��Ϊ��ɫ
		}
	}
	else if (GIMBAL_If_Big_Buff() == TRUE) //���ģʽ
	{
		if (IF_RC_SW1_UP) //˳ʱ��
		{
			if (Buff_Color == buff_blue)
			{
				Vision_Send_Data(VISION_BBUFF_CLOCKWISE); //��˳
				Buff_Type = VISION_BBUFF_CLOCKWISE;
			}
			else if (Buff_Color == buff_red)
			{
				Vision_Send_Data(VISION_RBUFF_CLOCKWISE); //��˳
				Buff_Type = VISION_RBUFF_CLOCKWISE;
			}
		}
		else if (IF_RC_SW1_DOWN) //��ʱ��
		{
			if (Buff_Color == buff_blue)
			{
				Vision_Send_Data(VISION_BBUFF_ANTI); //����
				Buff_Type = VISION_BBUFF_ANTI;
			}
			else if (Buff_Color == buff_red)
			{
				Vision_Send_Data(VISION_RBUFF_ANTI); //����
				Buff_Type = VISION_RBUFF_ANTI;
			}
		}
	}
	else
	{
		Vision_Send_Data(VISION_OFF);
		Buff_Type = VISION_OFF;
	}
}

/**
  * @brief  �������
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Ctrl(void)
{
	/* ȷ���з���ɫ */
	//s1��ʶ���,��ʶ����,�м䲻ʶ��
	if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_W)
	{
		Attack_Color_Choose = ATTACK_RED;
		flow_led_toggle(0);
		delay_ms(100);
		flow_led_toggle(0);
		delay_ms(100);
		flow_led_toggle(0);
		delay_ms(100); 
		flow_led_toggle(0);
		delay_ms(100);
	}
	else if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_S)
	{
		Attack_Color_Choose = ATTACK_BLUE;
		flow_led_on(0);
	}
	else if (IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_C)
	{
		Attack_Color_Choose = ATTACK_NONE;
		flow_led_off(0);
	}

	//��С���Է�����ɫʶ��ָ��
	if (Attack_Color_Choose == ATTACK_BLUE)
	{
		Vision_Send_Data(VISION_BLUE);
	}
	else if (Attack_Color_Choose == ATTACK_RED)
	{
		Vision_Send_Data(VISION_RED);
	}
	else if (Attack_Color_Choose == ATTACK_NONE)
	{
		Vision_Auto_Attack_Off();
	}
}

/**
  * @brief  �ر�����
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Auto_Attack_Off(void)
{
	Vision_Send_Data(VISION_OFF);
}

/*******************************�Ӿ�����ȡ*************************************/
/**
  * @brief  ��ȡyaw�������(x��)
  * @param  ���ָ��
  * @retval void
  * @attention  ���Ͻ�Ϊ0,������ʾĿ�����е����,������ʾ���ұ�
  */
void Vision_Error_Yaw(float *error)
{
	if (VisionRecvData.yaw_angle != 0)
	{
		//���Ϊ��ʱ��̨����,Ϊ��ʱ����
		*error = -(VisionRecvData.yaw_angle - VISION_MID_YAW);
	}
	else
	{
		*error = 0;
	}
}

/**
  * @brief  ��ȡpitch�������(y��)
  * @param  ���ָ��
  * @retval void
  * @attention  ���Ͻ�Ϊ0,������ʾĿ�����е��Ϸ�,������ʾ���·�
  */
void Vision_Error_Pitch(float *error)
{
	if (VisionRecvData.pitch_angle != 0)
	{
		*error = VisionRecvData.pitch_angle - VISION_MID_PITCH;
	}
	else
	{
		*error = 0;
	}
}
/*--------------------------------����ר�õ�yaw,pitch�����ǶȻ�ȡ-----------------------------*/
/**
  * @brief  ��ȡyaw���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������
  */
void Vision_Error_Angle_Yaw(float *error)
{
	//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
	*error = (VisionRecvData.yaw_angle + Vision_Comps_Yaw * VisionRecvData.distance / 100) / 60; //ת��Ϊ����
																						  //				* 8192.0f / 360.0f / 10.0f;//������Լ���ŷ���ǵķŴ������˶�Ӧ����
	if (VisionRecvData.yaw_angle == 0)													  //����
	{
		*error = 0;
	}
}

/**
  * @brief  ��ȡpitch���Ƕȣ�����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  �Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ
  */
float kvision_mouse_pitch = 0.007;
float mouse_pitch_comps = 0;		//�����Զʱ������겹��
float vision_pitch_dist = 2;		//�̶�����,�����˾��뿪�����벹��
float vision_pitch_dist_far = 4.4f; //�����˾��뿪����겹��
void Vision_Error_Angle_Pitch(float *error)
{

	//	if(GIMBAL_AUTO_PITCH_SB() == TRUE)  //�ж��Ƿ�����׼�ڱ�
	//	{
	//		*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch/SB_K_comps * VisionRecvData.distance/100)* 8192.0f / 360.0f / 10.0f;//��Ϊpitch�ǻ�еģʽ,���԰�ŷ����ת���ɻ�е��
	//	}
	// if(VisionRecvData.distance/100 >= vision_pitch_dist_far)                             //�жϾ��벹��
	//	{
	//		mouse_pitch_comps += MOUSE_Y_MOVE_SPEED * kvision_mouse_pitch;//ע������
	//		//�޷�����ֹ̫��
	//		mouse_pitch_comps = constrain_float(mouse_pitch_comps, -3, 0);
	//		*error = (VisionRecvData.pitch_angle
	//						+ Vision_Comps_Pitch * VisionRecvData.distance/100
	//							+ mouse_pitch_comps
	//				 )
	//				 * 8192.0f / 360.0f / 10.0f;//��Ϊpitch�ǻ�еģʽ,���԰�ŷ����ת���ɻ�е��
	//
	//	}
	//	else
	//	{
	*error = (VisionRecvData.pitch_angle + Vision_Comps_Pitch * VisionRecvData.distance / 100) / 150; //��Ϊpitch�ǻ�еģʽ,���԰�ŷ����ת���ɻ���
																							  //	}

	if (VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}

/*---------------------------------------------------------------���ר�� yaw��pitch�����ǶȻ�ȡ---------------------------------------------------*/

/**
  * @brief  ��ȡyaw���Ƕȣ����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������������ͷ����̨
  */
void Vision_Buff_Error_Angle_Yaw_Gimbal(float *error)
{
	//�Ӿ�������,�������̨���ƽǶ�ѡ������(����Ҽ�)
	*error = -VisionRecvData.yaw_angle; //ȡ����Ϊ�˶�Ӧyaw��ת��е������
	if (VisionRecvData.yaw_angle == 0)	//����
	{
		*error = 0;
	}
}

/**
  * @brief  ��ȡpitch���Ƕȣ����ר��
  * @param  ���ָ��
  * @retval void
  * @attention  �Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ
  */
void Vision_Buff_Error_Angle_Pitch(float *error)
{
	//�Ӿ��ϸ�����,ע����̨������̧ͷ���ǵ�ͷ(�ϼ��¼�)
	*error = VisionRecvData.pitch_angle; //pitch��̨���ұ�ʱ������ת��̨��е�Ǽ�С
	if (VisionRecvData.pitch_angle == 0)
	{
		*error = 0;
	}
}
/*---------------------------------------------------------------�������ר�� yaw��pitch�����ǶȻ�ȡ---------------------------------------------------*/
/**
  * @brief  ��ȡyaw������أ���ͷ�������ר��
  * @param  ���ָ��
  * @retval void
  * @attention  ������������ͷ����̨
  */
void Vision_Base_Yaw_Pixel(float *error)
{
	if (VisionRecvData.yaw_angle != 0)
	{
		//���Ϊ��ʱ��̨����,Ϊ��ʱ����
		*error = -(VisionRecvData.yaw_angle - 640);
	}
	else
	{
		*error = 0;
	}
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------*/

bool Get_Vision_distance(void) //TRUE Զ    FALSE ��
{
	if (VisionRecvData.distance > 400)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  ��ȡ����
  * @param  void
  * @retval void
  * @attention  
  */
void Vision_Get_Distance(float *distance)
{
	*distance = VisionRecvData.distance;
	if (VisionRecvData.distance < 0)
	{
		*distance = 0;
	}
}

/*----------------------------------------------------------------------------------�Ӿ���������--------------------------------------------------------------------------------------*/

/**
  * @brief  ������ɫ�ж�
  * @param  void
  * @retval
  * @attention  ����ϵͳ���ҵڶ����ƺ�Ϊʶ���,��Ϊʶ����,��ʶ�������˸
  */
uint8_t VISION_isColor(void)
{
	return Attack_Color_Choose;
}

/**
  * @brief  ���ָ���ж�
  * @param  void
  * @retval 
  * @attention  ����ϵͳ���ҵ������ƺ�Ϊ˳ʱ��,��Ϊ��ʱ��,��ʶ�������˸
  */
uint8_t VISION_BuffType(void)
{
	return Buff_Type;
}

/**
  * @brief  �жϷ��͵�ָ�����Ӿ����յ���ָ���Ƿ���ͬ
  * @param  void
  * @retval TRUEָ��һ��    FALSEָ�һ��
  * @attention  �Ӿ��յ�ʲôָ��,�ͷ�ͬ����ָ�����
  */
bool VISION_IfCmdID_Identical(void)
{
	if (VisionRecvData.CmdID == VisionSendHeader.CmdID)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  �ж��Ӿ����ݸ�������
  * @param  void
  * @retval TRUE������   FALSEû����
  * @attention  Ϊ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Get_New_Data��TRUE
  */
bool Vision_If_Update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  �Ӿ����ݸ��±�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void Vision_Clean_Update_Flag(void)
{
	Vision_Get_New_Data = FALSE;
}

/**
  * @brief  �жϻ�װ�װ�����
  * @param  void
  * @retval TRUE����   FALSEû��
  * @attention  Ϊ�Զ������׼��,���ڿ����ж�ÿ����һ����ͨ��У��,��Vision_Armor��TRUE
  */
bool Vision_If_Armor(void)
{
	return Vision_Armor;
}

/**
  * @brief  ��װ�ױ�־λ�ֶ���0(false)
  * @param  void
  * @retval void
  * @attention  �ǵ�Ҫ����,���������Լ�ѡ,���������������
  */
void Vision_Clean_Ammor_Flag(void)
{
	Vision_Armor = FALSE;
}