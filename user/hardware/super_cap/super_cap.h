#ifndef _CAP_TASK_H
#define _CAP_TASK_H

#include "main.h"
#include "stdbool.h"
#include "CAN_receive.h"


void Cap_Update_Cap_Inputvot(int16_t inputvot );
void Cap_Update_Cap_Capvot(int16_t capvot );
void Cap_Update_Cap_Test_current(int16_t current );
void Cap_Update_Cap_Target_Power(int16_t power );

void Cap_Init(void);
void Super_cap_task(void);
void CAN1_Cap_Send(uint16_t temPower);


#endif
