#include "chassis_behaviour.h"
#include "arm_math.h"
#include "gimbal_task.h"


extern  RC_ctrl_t rc_ctrl;

extern fp32 angle_swing_set;

void chassis_follow_gimbal_yaw_control(void)
{
    //ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
    static fp32 swing_time = 0.0f;
    //swing_time �Ǽ�������ĽǶ�
    static fp32 swing_angle = 0.0f;
    //max_angle ��sin�����ķ�ֵ
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //add_time ��ҡ�ڽǶȸı�Ŀ��������Խ��
    static fp32 const add_time = PI / 300.0f;
    //ʹ��ҡ�ڱ�־λ
    static uint8_t swing_flag = 0;

    //����ң������ԭʼ�����ź�
    //�ж��Ƿ�Ҫҡ��
    if (IF_RC_SW1_DOWN)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }
		
    //�жϼ��������ǲ����ڿ��Ƶ����˶����������˶���Сҡ�ڽǶ�
    if (rc_ctrl.rc.ch[2]>10 || rc_ctrl.rc.ch[3]>10)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
		
    //sin�������ɿ��ƽǶ�
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //sin����������2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }
		angle_swing_set = swing_angle;
}


