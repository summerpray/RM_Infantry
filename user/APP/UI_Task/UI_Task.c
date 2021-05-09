#include "UI_Task.h"
#include "main.h"
#include "RM_Cilent_UI.h"
#include "string.h"
#include "remote_control.h"
#include "CAN_Receive.h" //加载摩擦轮发射闭环真实转速
#include "super_cap.h"
#include "magazine.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#define PI 3.1415926
extern RC_ctrl_t rc_ctrl;
extern uint8_t Robot_ID;   //当前机器人的ID
extern uint16_t Cilent_ID; //发送者机器人对应的客户端ID

extern uint8_t Judge_Self_ID;		 //当前机器人的ID
extern uint16_t Judge_SelfClient_ID; //发送者机器人对应的客户端ID
extern float Supercap_Vot;			 //超级电容当前电量
extern int find_armor;
extern uint16_t Fric_enable;

Graph_Data G1, G2, G3, G4, G5, G6, G7, G8, G9, G10, G11;
Graph_Data G_BLOCK1, G_BLOCK2, G_BLOCK3, G_BLOCK4, G_BLOCK5, G_BLOCK6, G_BLOCK7, G_BLOCK8, G_BLOCK9;
String_Data CH_SHOOT;
String_Data CH_FLRB;
String_Data CH_SUPERCAP;
String_Data CH_MODE;
String_Data CH_AUTO;
String_Data CH_MAG;
Float_Data FLOAT_SUPER;
char shoot_arr[5] = "SHOOT";
char flrb_arr[4] = "FRBL";
char super_arr[6] = "SUPER:";
char mode_arr[6] = "NORMOL";
char auto_arr[4] = "AUTO";
char mag_arr[7] = "MAGZINE";

int last_length;
int capline_length = 0;
void UI_Task(void *pvParameters)
{
	vTaskDelay(1000);
	determine_ID();
	Robot_ID = Judge_Self_ID;
	Cilent_ID = Judge_SelfClient_ID;

	memset(&G1, 0, sizeof(G1));				//中心垂线
	memset(&G2, 0, sizeof(G2));				//上击打线
	memset(&G3, 0, sizeof(G3));				//中心水平线
	memset(&G4, 0, sizeof(G4));				//枪管轴心线
	memset(&G5, 0, sizeof(G5));				//下击打线
	memset(&G6, 0, sizeof(G6));				//远距离击打线
	memset(&G7, 0, sizeof(G7));				//摩擦轮状态
	memset(&G8, 0, sizeof(G8));				//中央瞄准圈
	memset(&G9, 0, sizeof(G9));				//自瞄状态
	memset(&G10, 0, sizeof(G10));			//自瞄状态
	memset(&CH_SHOOT, 0, sizeof(CH_SHOOT)); //摩擦轮标识
	memset(&CH_MODE, 0, sizeof(CH_MODE));	//模式标识
	memset(&CH_MAG, 0, sizeof(CH_MAG));		//弹仓标识

	static double angle = 0;
	/*绘制中心瞄准线*/
	Line_Draw(&G1, "001", UI_Graph_ADD, 9, UI_Color_Cyan, 1, 960, 330, 960, 620);
	Line_Draw(&G2, "002", UI_Graph_ADD, 9, UI_Color_Cyan, 1, 880, 580, 1040, 580);
	Line_Draw(&G3, "003", UI_Graph_ADD, 9, UI_Color_Cyan, 1, 800, 540, 1120, 540);
	Line_Draw(&G4, "004", UI_Graph_ADD, 9, UI_Color_Cyan, 1, 880, 500, 1040, 500);
	Line_Draw(&G5, "005", UI_Graph_ADD, 9, UI_Color_Cyan, 1, 900, 420, 1020, 420);
	Line_Draw(&G6, "006", UI_Graph_ADD, 9, UI_Color_Cyan, 1, 920, 370, 1000, 370);

	/*绘制中央瞄准圈*/
	Circle_Draw(&G8, "007", UI_Graph_ADD, 8, UI_Color_Green, 3, 960 + (int)340 * sin((angle)*2 * PI / 360.0), 200 + (int)340 * cos((angle)*2 * PI / 360.0), 50);
	/*绘制当前模式字符*/
	Char_Draw(&CH_MODE, "009", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 6, 4, 80, 840, &mode_arr[0]);

	/*绘制SHOOT字符*/
	Char_Draw(&CH_SHOOT, "010", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 5, 4, 80, 840, &shoot_arr[0]);
	/*绘制SHOOT指示灯(摩擦轮状态)*/
	Circle_Draw(&G7, "011", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 230, 830, 10);

	/*绘制当前自瞄情况字符*/
	Char_Draw(&CH_AUTO, "012", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 4, 4, 80, 790, &auto_arr[0]);
	/*绘制自瞄情况指示灯*/
	Circle_Draw(&G9, "013", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 230, 780, 10);

	/*绘制弹仓情况字符*/
	Char_Draw(&CH_MAG, "014", UI_Graph_ADD, 9, UI_Color_Yellow, 20, 7, 4, 80, 740, &mag_arr[0]);
	/*绘制弹仓情况指示灯*/
	Circle_Draw(&G10, "015", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 250, 730, 10);

	Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
	Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
	Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
	Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
	Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
	Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
	Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
	Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
	Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);

	while (1)
	{
		if (Magazine_IfOpen() == TRUE)
		{
			Circle_Draw(&G10, "015", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 250, 730, 10);
		}
		else if (Magazine_IfOpen() == FALSE)
		{
			Circle_Draw(&G10, "015", UI_Graph_Del, 9, UI_Color_Yellow, 15, 250, 730, 10);
		}
		if (IF_MOUSE_PRESSED_RIGH)
		{
			if (find_armor == 1)
			{
				Circle_Draw(&G9, "013", UI_Graph_ADD, 9, UI_Color_Purplish_red, 15, 230, 780, 10);
			}
			else
			{
				Circle_Draw(&G9, "013", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 230, 780, 10);
			}
		}
		else
		{
			Circle_Draw(&G9, "013", UI_Graph_Del, 9, UI_Color_Yellow, 15, 230, 780, 10);
		}
		if (Fric_enable)
		{
			Circle_Draw(&G7, "011", UI_Graph_ADD, 9, UI_Color_Yellow, 15, 230, 830, 10);
		}
		else
		{
			Circle_Draw(&G7, "011", UI_Graph_Del, 9, UI_Color_Yellow, 15, 230, 830, 10);
		}

		UI_ReFresh(7, G_BLOCK1, G_BLOCK2, G_BLOCK3, G_BLOCK4, G_BLOCK5, G_BLOCK6, G_BLOCK7);
		UI_ReFresh(2, G_BLOCK8, G_BLOCK9);
		UI_ReFresh(7, G1, G2, G3, G4, G5, G6, G7);
		UI_ReFresh(1, G8);

		Char_ReFresh(CH_SHOOT);

		Char_ReFresh(CH_AUTO);
		UI_ReFresh(1, G9);

		Char_ReFresh(CH_MAG);
		UI_ReFresh(1, G10);
		capline_length = (int)(Supercap_Vot - 22);
		capline_length = 0;
		if (capline_length == 9)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 8)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 7)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 6)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 5)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 4)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_ADD, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 3)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Purplish_red, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Purplish_red, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_ADD, 9, UI_Color_Purplish_red, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 2)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Purplish_red, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_ADD, 9, UI_Color_Purplish_red, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 1)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_ADD, 9, UI_Color_Purplish_red, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		else if (capline_length == 0)
		{
			Rectangle_Draw(&G_BLOCK1, "016", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1600, 790, 1630, 800);
			Rectangle_Draw(&G_BLOCK2, "017", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1630, 790, 1660, 800);
			Rectangle_Draw(&G_BLOCK3, "018", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1660, 790, 1690, 800);
			Rectangle_Draw(&G_BLOCK4, "019", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1690, 790, 1720, 800);
			Rectangle_Draw(&G_BLOCK5, "020", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1720, 790, 1750, 800);
			Rectangle_Draw(&G_BLOCK6, "021", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1750, 790, 1780, 800);
			Rectangle_Draw(&G_BLOCK7, "022", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1780, 790, 1810, 800);
			Rectangle_Draw(&G_BLOCK8, "023", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1810, 790, 1840, 800);
			Rectangle_Draw(&G_BLOCK9, "024", UI_Graph_Del, 9, UI_Color_Cyan, 10, 1840, 790, 1870, 800);
		}
		last_length = capline_length;
		//频率控制10hz
		vTaskDelay(33);
	}
}
