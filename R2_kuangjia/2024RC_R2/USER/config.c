#include "config.h"


/**
 * @brief 电机类型选择,请跳转到config.c中配置
 * @param 电机类型
 * @return NULL
*/
void RM_Motor_Init(void)
{
	RM_BOARD_MSG_1[0].MOTOR_ID = 0x01;
	RM_BOARD_MSG_1[1].MOTOR_ID = 0x02;
	RM_BOARD_MSG_1[2].MOTOR_ID = 0x03;
	RM_BOARD_MSG_1[3].MOTOR_ID = 0x04;
	RM_BOARD_MSG_1[4].MOTOR_ID = 0x05;
	RM_BOARD_MSG_1[5].MOTOR_ID = 0x06;
	RM_BOARD_MSG_1[6].MOTOR_ID = 0x07;
	RM_BOARD_MSG_1[7].MOTOR_ID = 0x08;
	
	
	RM_BOARD_MSG_1[0].MOTOR_TYPE = M_3508;
	RM_BOARD_MSG_1[1].MOTOR_TYPE = M_3508;
	RM_BOARD_MSG_1[2].MOTOR_TYPE = M_3508;
    RM_BOARD_MSG_1[3].MOTOR_TYPE = M_3508;
	RM_BOARD_MSG_1[4].MOTOR_TYPE = M_2006;
    RM_BOARD_MSG_1[5].MOTOR_TYPE = M_3508;
	RM_BOARD_MSG_1[6].MOTOR_TYPE = M_3508;
	RM_BOARD_MSG_1[7].MOTOR_TYPE = M_3508;

    BOARD_RM_MOTOR_INIT(BOARD_RM1,RM_BOARD_MSG_1[0].MOTOR_TYPE,RM_BOARD_MSG_1[1].MOTOR_TYPE,RM_BOARD_MSG_1[2].MOTOR_TYPE,
                                  RM_BOARD_MSG_1[3].MOTOR_TYPE,RM_BOARD_MSG_1[4].MOTOR_TYPE,RM_BOARD_MSG_1[5].MOTOR_TYPE,
                                  RM_BOARD_MSG_1[6].MOTOR_TYPE,RM_BOARD_MSG_1[7].MOTOR_TYPE);
}


void rm_motor_control(RM_BRD_MSG* MOTOR, BOARD_NUM board_id)
{
	switch (MOTOR->MOTOR_MODE)
	{
		case SPEED_CONTROL_MODE:
		{
			BOARD_RM_SPEED_CONTROL(board_id,MOTOR->MOTOR_ID,MOTOR->TARGET_RPM);
			break;
		}

		case POSITION_CONTROL_MODE:
		{
			BOARD_RM_POSITION_CONTROL(board_id,MOTOR->MOTOR_ID,MOTOR->TARGET_POS);
			break;
		}

		case HOMEING_MODE:
		{
			BOARD_RM_HOMEING_MODE(board_id,MOTOR->MOTOR_ID,MOTOR->TARGET_RPM,MOTOR->TARGET_TARQUE);
			break;
		}

		case MOTO_OFF:
		{
			BOARD_RM_MOTOR_OFF(board_id,MOTOR->MOTOR_ID);
			break;
		}
		
		case MOTOR_ON:
		{
			BOARD_RM_MOTOR_ON(board_id,MOTOR->MOTOR_ID);
			break;
		}

		case VELOCITY_PLANNING_MODE:
		{
			BOARD_RM_VEL_TPLAN(board_id,MOTOR->MOTOR_ID,MOTOR->velocity_plan.Pstart,MOTOR->velocity_plan.Pend,
								MOTOR->velocity_plan.Vstart,MOTOR->velocity_plan.Vmax,MOTOR->velocity_plan.Vend,
								MOTOR->velocity_plan.Rac,MOTOR->velocity_plan.Rde);
			break;
		}

		default:
			break;
	}
}


/**
 * @brief PID初始化函数,请跳转到config.c中配置
*/
void Pid_Init_All(void)
{
	static int i=0;
	if(i==0)
	{
		BOARD_RM_VEL_PID(BOARD_RM1,0x01,12.0,0.4,10000,5);
		BOARD_RM_VEL_PID(BOARD_RM1,0x02,12.0,0.4,10000,5);
	}
    
	if(i==1)
	{
		BOARD_RM_VEL_PID(BOARD_RM1,0x03,12.0,0.4,10000,5);
		BOARD_RM_VEL_PID(BOARD_RM1,0x04,12.0,0.4,10000,5);
	}
	
	if(i==2)
	{
		BOARD_RM_VEL_PID(BOARD_RM1,0x05,12.0,0.4,10000,5);
		BOARD_RM_VEL_PID(BOARD_RM1,0x06,12.0,0.4,10000,5);
	}
	
	if(i==3)
	{
		BOARD_RM_VEL_PID(BOARD_RM1,0x07,12.0,0.4,10000,5);
		BOARD_RM_VEL_PID(BOARD_RM1,0x08,12.0,0.4,10000,5);
	}
	
	i++;
	if(i>3)
		i=0;
}


//测试函数
void RM_SPPED_TEST(void)
{
	int motor_num = 4;
    static int16_t index=0;
    for(int16_t i=0; i<2; i++)
    {
        rm_motor_control(&RM_BOARD_MSG_1[index+i],BOARD_RM1);
    }
    
    index+=2;
    if(index >= motor_num)
        index = 0;
}


void VESC_Init(void)
{
    VESC_BOARD_MSG_1[0].MOTOR_ID = 101;            //电调ID采用 101，102，103.....
    VESC_BOARD_MSG_1[1].MOTOR_ID = 102;
    VESC_BOARD_MSG_1[0].MOTOR_ID = 103;            
    VESC_BOARD_MSG_1[1].MOTOR_ID = 104;
    VESC_BOARD_MSG_1[2].MOTOR_ID = 105;
    VESC_BOARD_MSG_1[3].MOTOR_ID = 106;
}
