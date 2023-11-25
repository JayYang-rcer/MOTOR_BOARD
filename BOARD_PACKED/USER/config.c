#include "config.h"

/**
 * @brief 电机类型选择,请跳转到config.c中配置。默认为3508电机
 * @param 电机类型
 * @return NULL
*/
void Motor_Type_Init(void)
{
	MOTO_REAL_INFO[0].Motor_Type = M_3508;
	MOTO_REAL_INFO[1].Motor_Type = M_3508;
	MOTO_REAL_INFO[2].Motor_Type = M_3508;
    MOTO_REAL_INFO[3].Motor_Type = M_3508;
    MOTO_REAL_INFO[4].Motor_Type = M_3508;
	MOTO_REAL_INFO[5].Motor_Type = M_3508;
	MOTO_REAL_INFO[6].Motor_Type = M_3508;
    MOTO_REAL_INFO[7].Motor_Type = M_3508;
}


/**
 * @brief PID初始化函数,请跳转到config.c中配置。默认配置
*/
void Pid_Init_All(void)
{
    PID_Parameter_Init(&MOTOR_PID_RPM[0],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x201号电机 M2006
	PID_Parameter_Init(&MOTOR_PID_RPM[1],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x202号电机 M3508
	PID_Parameter_Init(&MOTOR_PID_RPM[2],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x203号电机 M3508
	PID_Parameter_Init(&MOTOR_PID_RPM[3],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x204号电机 M3508
    PID_Parameter_Init(&MOTOR_PID_RPM[4],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x205号电机 M2006
	PID_Parameter_Init(&MOTOR_PID_RPM[5],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x206号电机 M3508
	PID_Parameter_Init(&MOTOR_PID_RPM[6],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x207号电机 M3508
	PID_Parameter_Init(&MOTOR_PID_RPM[7],12.0,0.4,0.1 , 10000 ,10 , 10000 , 500);	//0x208号电机 M3508

    PID_Parameter_Init(&MOTOR_PID_POS[0], 10.2, 0, 0.01, 7000, 1, 7000, 500);   //位置环PID
    PID_Parameter_Init(&MOTOR_PID_POS[1], 10.2, 0, 0.01, 7000, 1, 7000, 500);
    PID_Parameter_Init(&MOTOR_PID_POS[2], 10.2, 0, 0.01, 7000, 1, 7000, 500);
    PID_Parameter_Init(&MOTOR_PID_POS[3], 10.2, 0, 0.01, 7000, 1, 7000, 500);
    PID_Parameter_Init(&MOTOR_PID_POS[4], 10.2, 0, 0.01, 7000, 1, 7000, 500);   
    PID_Parameter_Init(&MOTOR_PID_POS[5], 10.2, 0, 0.01, 7000, 1, 7000, 500);
    PID_Parameter_Init(&MOTOR_PID_POS[6], 10.2, 0, 0.01, 7000, 1, 7000, 500);
    PID_Parameter_Init(&MOTOR_PID_POS[7], 10.2, 0, 0.01, 7000, 1, 7000, 500);
}


/**
 * @brief VESC驱动器的ID号设置，写死别动。具体ID号要在驱动器上位机进行配置，此处需要与其保持一致
*/
void VESC_Init(void)
{
    VESC_MOTO_INFO[0].MotorType = VESC_5065;                 //没啥用，图个乐子
    VESC_MOTO_INFO[0].VESC_CAN_ID = VESC_CAN_ID1;            //电调ID采用 101，102，103.....

    VESC_MOTO_INFO[1].MotorType = VESC_5065;
    VESC_MOTO_INFO[1].VESC_CAN_ID = VESC_CAN_ID2;

    VESC_MOTO_INFO[2].MotorType = VESC_5065;
    VESC_MOTO_INFO[2].VESC_CAN_ID = VESC_CAN_ID3;

    VESC_MOTO_INFO[3].MotorType = VESC_5065;
    VESC_MOTO_INFO[3].VESC_CAN_ID = VESC_CAN_ID4;

    VESC_MOTO_INFO[4].MotorType = VESC_5065;
    VESC_MOTO_INFO[4].VESC_CAN_ID = VESC_CAN_ID5;

    VESC_MOTO_INFO[5].MotorType = VESC_5065;
    VESC_MOTO_INFO[5].VESC_CAN_ID = VESC_CAN_ID6;
}
