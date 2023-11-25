#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

//电机板ID枚举
typedef enum BOARD_NUM
{
    BOARD_RM1=1,
    BOARD_RM2,
    BOARD_VESC1,
    BOARD_VESC2
}BOARD_NUM;

typedef struct CONTROL_S
{
    int32_t MOTOR_TAR_VALUE[8]; //电机的目标参数，各种
    int32_t MOTOR_TARQUE[8];    //电机力矩
    uint8_t CAN1_RECIEVE_FLAG;  //是否接收到can信号
    uint8_t CAN_TIMER;  //判断主板can信号是否断开
    uint8_t BOARD_ID;   //电极板ID，封装为枚举了
}CONTROL_S;

void rm_motor_control_comm(CAN_RxHeaderTypeDef* CAN, uint8_t* RxData);
void vesc_motor_control_comm(CAN_RxHeaderTypeDef* CAN, uint8_t* RxData);
void can_loss(void);
void Send_Rm_Motor_Msg(void);
void Send_Vesc_Motor_Msg(void);

extern CONTROL_S CONTROL;

#endif
