#ifndef __RM_BOARD_ELMO_H
#define __RM_BOARD_ELMO_H

#include "main.h"


/** 
  * @brief T型速度规划结构体  
  * @note     
*/
typedef struct VELOCITY_PLANNING //速度规划
{
	float Pstart;        //开始位置
	float Pend;          //结束位置
	uint16_t Vstart;        //开始的速度           // 单位：RPM 绝对值
	uint16_t Vmax;          //最大的速度
	uint16_t Vend;          //末尾的速度
	float Rac;           //加速路程的比例
	float Rde;           //减速路程的比例
}VELOCITY_PLANNING;


/** 
  * @brief  电机种类  M3508和M2006
*/ 
typedef enum
{
	M_3508 = 17,
	M_2006,
	M_6020,
	VESC_5065,
	NONE  //表示没有接入电机
}MotorType_TypeDef;

typedef enum BOARD_NUM
{
    BOARD_RM1=1,
    BOARD_RM2,
    BOARD_VESC1,
    BOARD_VESC2
}BOARD_NUM;

//驱动器工作模式
typedef enum MOTOR_MODE_E
{
	VESC_SPEED = 0,		//速度环模式
	VESC_CURRENT,		//电流环模式
	VESC_DUTY,			//占空比驱动
	VESC_POS,				//位置模式


	SPEED_CONTROL_MODE = 4,
	VELOCITY_PLANNING_MODE,
	CURRENT_MODE,
	POSITION_CONTROL_MODE,
	HOMEING_MODE=10,
    MOTO_OFF,
    VEL_PID_CONFIG,
    POS_PID_CONFIG,
    MOTOR_ON,
    MOTOR_TYPE_CONFIG,
    VELOCITY_PLANNING_MODE_CFG2
}MOTOR_MODE_E;


typedef struct RM_BRD_MSG
{
    MotorType_TypeDef MOTOR_TYPE;
    uint16_t MOTOR_ID;
    MOTOR_MODE_E MOTOR_MODE;
    int16_t TARGET_RPM;
    uint16_t TARGET_TARQUE;
    float TARGET_POS;

    VELOCITY_PLANNING velocity_plan;

    float REAL_ANGLE;
    int16_t RPM;
    int16_t CURRETN;
}RM_BRD_MSG;


typedef struct VESC_BRD_MSG
{
    uint16_t MOTOR_ID;
    MOTOR_MODE_E MOTOR_MODE;
    int32_t TARGET_eRPM;
    float   TARGET_DUTY;
    float   TARGET_CURRENT;
    float   TARGET_POS;

    float DUTY;
    float RPM;
    float CURRETN;
    float POS;
}VESC_BRD_MSG;


void BOARD_RM_MOTOR_INIT(BOARD_NUM NUM, MotorType_TypeDef MODE1,MotorType_TypeDef MODE2,MotorType_TypeDef MODE3,
                                        MotorType_TypeDef MODE4,MotorType_TypeDef MODE5,MotorType_TypeDef MODE6,
                                        MotorType_TypeDef MODE7,MotorType_TypeDef MODE8);
void BOARD_RM_SPEED_CONTROL(BOARD_NUM NUM, uint16_t controller_id, int16_t target_rpm);
void BOARD_RM_POSITION_CONTROL(BOARD_NUM NUM, uint16_t controller_id, int16_t target_pos);
void BOARD_RM_HOMEING_MODE(BOARD_NUM NUM, uint16_t controller_id, int16_t target_rpm, int16_t target_tarque);
void BOARD_RM_VEL_PID(BOARD_NUM NUM ,uint16_t controller_id, float Kp, float Ki, uint16_t Out_Max, int16_t Dead_Size);
void BOARD_RM_POS_PID(BOARD_NUM NUM, uint16_t controller_id, float Kp, float Kd, uint16_t Out_Max, int16_t Dead_Size);
void BOARD_RM_VEL_TPLAN(BOARD_NUM NUM, uint16_t controller_id, int16_t Pstart, int16_t Pend, uint16_t Vstart, uint16_t Vmax, uint16_t Vend, float Rac, float Rde);
void BOARD_RM_MOTOR_ON(BOARD_NUM NUM, uint16_t controller_id);
void BOARD_RM_MOTOR_OFF(BOARD_NUM NUM, uint16_t controller_id);
void rm_motor_control(RM_BRD_MSG* MOTOR, BOARD_NUM board_id);

void BOARD_VESC_SPEED_CONTROL(BOARD_NUM NUM, uint16_t controller_id, int32_t target_rpm);
void BOARD_VESC_SET_DUTY(BOARD_NUM NUM, uint16_t controller_id, float duty);
void BOARD_VESC_SET_CURRENT(BOARD_NUM NUM, uint16_t controller_id, float current);
void BOARD_VESC_SET_POS(BOARD_NUM NUM, uint16_t controller_id, float pos);

void RM_SPPED_TEST(void);

void get_rm_board_msg(CAN_RxHeaderTypeDef *CAN, uint8_t* RxData);
void get_vesc_board_msg(CAN_RxHeaderTypeDef *CAN, uint8_t* RxData);
int16_t board_id_choose(BOARD_NUM NUM, uint16_t controller_id);

extern RM_BRD_MSG RM_BOARD_MSG_1[8];
extern RM_BRD_MSG RM_BOARD_MSG_2[8];
extern VESC_BRD_MSG VESC_BOARD_MSG_1[6];
extern VESC_BRD_MSG VESC_BOARD_MSG_2[6];

#endif
