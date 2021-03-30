/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       start_task.c/h
  * @brief      �������񣬽�һ������������������Դ�������������ȼ�,
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

#include "Start_Task.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"

#include "User_Task.h"
#include "Chassis_Task.h"
#include "Gimbal_Task.h"
#include "delay.h"
#include "led.h"
#include "remote_control.h"
#include "stdbool.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "magazine.h"

#include "timer.h"
#define GIMBAL_TASK_PRIO 19
#define GIMBAL_STK_SIZE 512
TaskHandle_t GIMBALTask_Handler;

#define Chassis_TASK_PRIO 18
#define Chassis_STK_SIZE 512
TaskHandle_t ChassisTask_Handler;


#define shoot_TASK_PRIO 17
#define shoot_STK_SIZE 512
TaskHandle_t shoot_Task_Handler;

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;


#define LED0_TASK_PRIO 6
#define LED0_STK_SIZE 512
static TaskHandle_t LED0Task_Handler;

#define Task_10ms_TASK_PRIO 5
#define Task_10ms_STK_SIZE 512
static TaskHandle_t Task_10msTask_Handler;

#define System_control_TASK_PRIO 7
#define System_control_STK_SIZE 512
static TaskHandle_t System_controlTask_Handler;




extern RC_ctrl_t rc_ctrl;

//static RC_TEST_t RC_TEST;


void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();


    xTaskCreate((TaskFunction_t)GIMBAL_task,
                (const char *)"GIMBAL_task",
                (uint16_t)GIMBAL_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)GIMBAL_TASK_PRIO,
                (TaskHandle_t *)&GIMBALTask_Handler);

    xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);

     xTaskCreate((TaskFunction_t)shoot_task,
                (const char *)"shoot_task",
                (uint16_t)shoot_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)shoot_TASK_PRIO,
                (TaskHandle_t *)&shoot_Task_Handler);
								
     xTaskCreate((TaskFunction_t)LED0,
                (const char *)"LED0",
                (uint16_t)LED0_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)LED0_TASK_PRIO,
                (TaskHandle_t *)&LED0Task_Handler);

     xTaskCreate((TaskFunction_t)Task_10ms,
                (const char *)"Task_10ms",
                (uint16_t)Task_10ms_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Task_10ms_TASK_PRIO,
                (TaskHandle_t *)&Task_10msTask_Handler);
								
		 xTaskCreate((TaskFunction_t)System_control,
                (const char *)"System_control",
                (uint16_t)System_control_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)System_control_TASK_PRIO,
                (TaskHandle_t *)&System_controlTask_Handler);


    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}

void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //������
                (const char *)"start_task",          //��������
                (uint16_t)START_STK_SIZE,            //�����ջ��С
                (void *)NULL,                        //���ݸ��������Ĳ���
                (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
                (TaskHandle_t *)&StartTask_Handler); //������
}




void LED0(void *pvParameters)
{
	while(1)
	{
		if(SYSTEM_GetSystemState( )==SYSTEM_RUNNING)
		{
			if(SYSTEM_GetRemoteMode()==RC)
		  	{
          flow_led_on(0);
				  flow_led_off(1);
					
		  	}
      if(SYSTEM_GetRemoteMode()==KEY)
			  {
          flow_led_on(1);
					flow_led_off(0);
			  }
		 
	 }
   else
	 {
          flow_led_off(0);
		      flow_led_off(1);
          flow_led_off(2);
		      flow_led_off(3);	
          flow_led_off(4);
		      flow_led_off(5);
          flow_led_off(6);
		      flow_led_off(7);		 
	 }
vTaskDelay(TIME_STAMP_1MS);
  }
	
}


//ÿ10msִ��һ��������
void Task_10ms(void *pvParameters)
{
	for(;;)
	{	
		vTaskDelay(TIME_STAMP_10MS);				//10ms
//���벿��		
		Magazine_Ctrl();		//���ֿ���
		
//		if(IF_RC_SW1_UP)
//		 {
//		   TIM4_FrictionPwmOutp(rc_ctrl.rc.ch[3],0);
//	   }
//		else
//		 {
//		   TIM4_FrictionPwmOutp(0,-rc_ctrl.rc.ch[3]); 
//    	}
		
	}
}

/*******************************************************/
/*ϵͳ״̬���£�������ʱ��ϵͳ����*/
//����ģʽ
eRemoteMode remoteMode = RC;

//ϵͳ״̬
eSystemState systemState = SYSTEM_STARTING;

//���ؿ���ģʽ
eRemoteMode SYSTEM_GetRemoteMode( )
{
	return remoteMode;
}

//����ϵͳ״̬
eSystemState SYSTEM_GetSystemState( )
{
	return systemState;
}


//ϵͳ����״̬������
void System_control(void *pvParameters)
{
	//static portTickType currentTime;
	portTickType currentTime;
	for(;;)
	{
		currentTime = xTaskGetTickCount();	//��ȡ��ǰϵͳʱ��	
   if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//��ʼ��ģʽ
		{
		  MPU_Update_last();
      system_update();
		}
		else
		{
      GIMBAL_MPU_Update();
		  Gimbal_Error_Read();
		}
		remote_StateControl();
//		vTaskDelayUntil(&currentTime, TIME_STAMP_1MS);//������ʱ
  }

}


//����ϵͳ������ʱ2500ms
void system_update(void)
{
	static uint32_t  ulInitCnt  =  0;
	
	if (systemState == SYSTEM_STARTING)
	{
		ulInitCnt++;

		if (ulInitCnt > 3500)//������ʱ,1ms*2k=2s,Ϊ�˸�MPU����ʱ��
		{
			ulInitCnt = 0;

			systemState = SYSTEM_RUNNING;//�������,ת������ͨģʽ
		}
	}
}

void remote_StateControl(void)
{
	
	if(IF_RC_SW2_UP)
	{
		remoteMode=KEY;
	}
	else
	{
		remoteMode=RC;
	}
	
}
/*******************************************************/


/*******************************************************/

/*����ָ�뷽ʽ����ң��������*/

//static void test_init(RC_TEST_t *test_init);
//static void test_mode(RC_TEST_t *test_mode);

//void test_init(RC_TEST_t *test_init)
//{
//	test_init->rc_test=get_remote_control_point();
//}

//void test_mode(RC_TEST_t *test_mode )
//{
//	
//	if(test_mode->rc_test->rc.s[0]==1)
//	{
//		led_green_on();
//		delay_ms(500);
//		led_green_off();
//	}
//	
//		if(test_mode->rc_test->rc.s[0]==2)
//	{
//		led_red_on();
//		delay_ms(500);
//		led_red_off();
//	}
//	
//}

//void LED0(void *pvParameters)
//{
//	while(1)
//	 {
//     test_init(&RC_TEST);
//	   test_mode(&RC_TEST);
//	 }
//}

/*******************************************************/
