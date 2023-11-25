#ifndef __RM_MOTOR_H
#define __RM_MOTOR_H

#include "main.h"
#define ABS(x)      ((x)>0? (x):(-(x)))


typedef struct GM6020
{
    uint16_t can_id;
    int16_t  set_voltage;       //设置电压
    uint16_t rotor_angle;       //转子角度
    int16_t  rotor_speed;       //转子速度
    int16_t  torque_current;    //力矩电流
    uint8_t  temp;              //电机温度
}GM_6020;



//驱动器工作模式
typedef enum CONTROL_COMM
{
	VESC_SPEED = 0,		//速度环模式
	VESC_CURRENT,		//电流环模式
	VESC_DUTY,			//占空比驱动
	VESC_POS,				//位置模式


/******************************************************************
以上为VESC驱动器的配置使用

以下为大疆电机的配置使用
*******************************************************************/
	SPEED_CONTROL_MODE = 4,	//速度模式
	VELOCITY_PLANNING_MODE,
	CURRENT_MODE,
	POSITION_CONTROL_MODE,
	SPEED_TARQUE_CONTROL_MODE,
	POSITION_TORQUE_MODE,
	HOMEING_MODE,
    MOTO_OFF,
    VEL_PID_CONFIG,
    POS_PID_CONFIG,
    MOTOR_ON,
    MOTOR_TYPE_INIT,
	VELOCITY_PLANNING_MODE_CFG2
}CONTROL_COMM;

/** 
  * @brief  电机种类  M3508和M2006
*/ 
typedef enum
{
	M_3508=17,
	M_2006,
	M_6020,
	VESC_5065,
	NONE  //表示没有接入电机
}MotorType_TypeDef;



//大疆电机CAN_ID
typedef enum RM_MOTOR_CAN_ID
{
    CAN_CHASSIS_ALL_ID = 0x200,
    M3508_M1_ID = 0x201,
    M3508_M2_ID = 0x202,
    M3508_M3_ID = 0x203,
    M3508_M4_ID = 0x204,
	M3508_M5_ID = 0x205,
	M3508_M6_ID = 0x206,
	M3508_M7_ID = 0x207,
	M3508_M8_ID = 0x208,
}RM_MOTOR_CAN_ID;


//VESC电调CAN_ID，写死
typedef enum
{
	VESC_CAN_ID1 = 101,
	VESC_CAN_ID2 = 102,
	VESC_CAN_ID3 = 103,
	VESC_CAN_ID4 = 104,
	VESC_CAN_ID5 = 105,
	VESC_CAN_ID6 = 106,
}VESC_CAN_ID;



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
	int flag;            //完成标志位，电机停下来的时候置1
}VELOCITY_PLANNING;



/**
 * @brief 回零模式结构体。说实在，这个模式和速度转矩没什么差别。直接用速度转矩即可
*/
typedef struct
{
	float current;
	float Vel;				//目标速度
	float output;
	int16_t  TARGET_TORQUE;//目标转矩，用电流表示
	int flag;
	int32_t cnt;
}HOMING_MODE_TYPE;



/** 
  * @brief  速度转矩模式结构体  
*/
typedef struct
{
	float Current;
	float Target_Vel;//目标速度
	float Output;
	int16_t  TARGET_TORQUE; //目标转矩，用电流表示
	int Flag;
	int32_t Cnt;
}VELOCITY_TARQUE_TYPDEF;



typedef struct
{
	uint32_t Motor_Mode;//电机模式
						//POSITION_CONTROL_MODE位置模式
						//POSITION_TARQUE_CONTROL_MODE位置_力度模式
						//SPEED_TARQUE_CONTROL_MODE位置_力度模式
						//SPEED_CONTROL_MODE速度模式
						//MOTO_OFF电机关闭-->电流不发送
						//VELOCITY_PLANNING_MODE梯形规划模式
	
	MotorType_TypeDef Motor_Type;

    GM_6020 GM6020;
	
    uint16_t  	ANGLE;            	// 采样转子角度
    int16_t  	RPM;				// 实际转子转速
    int16_t  	CURRENT;			// 实际转矩电流
    int16_t  	TARGET_CURRENT;		// 目标转矩电流

	int16_t  TARGET_POS;		//目标角度(位置)
	int16_t    TARGET_RPM;		//目标转速
	int      Velflag;			//数度为零时，置1 
	int 	 PosFlag;			//未达指定位置，置0，到达置1
	
	

	VELOCITY_PLANNING 		Velocity_Planning;	//速度规划
	HOMING_MODE_TYPE 		HomingMode;			//电机回零模式
	VELOCITY_TARQUE_TYPDEF  Velocity_Tarque;	//速度转矩结构体



	// 角度积分时用到下面变量
	float		REAL_ANGLE;         //处理过的真实角度（必须用float）
	uint8_t	 	FIRST_ANGLE_INTEGRAL_FLAG;  //是否第一次进行角度积分
	uint16_t 	LAST_ANGLE;   
	int16_t 	Filter_RPM;
}MOTOR_REAL_INFO;


//VESC驱动抽象结构体
typedef struct 
{
	MotorType_TypeDef MotorType;//电机类型	

	uint8_t VESC_CAN_ID;		//电调ID采用 101，102，103.....
	CONTROL_COMM MOTOR_MODE;	//由于电机模式一般固定，所以直接在config.c初始化

	float NOW_ANGLE;			//当前角度
	float NOW_eRPM;				//当前的电转速
	float NOW_CURRENT;			//当前电流
	float NOW_VESC_DUTY;		//当前驱动器占空比

	float TARGET_ANGLE;			//目标角度(位置)
	float TARGET_eRPM;			//目标电转速
	float TARGET_CURRENT;		//目标电流(单位为A)
	float TARGET_VESC_DUTY;		//目标占空比(0 <= duty <= 1)

}VESC_MOTOR_INFO;



void Velocity_Planning_MODE(MOTOR_REAL_INFO *M3508_MOTOR);
void Velocity_Planning_setpos(MOTOR_REAL_INFO *M3508_MOTOR,float Pstart,float Pend,uint16_t Vstart,uint16_t Vmax,uint16_t Vend,float Rac,float Rde);
float Max_Value_Limit(float Value, float Limit);
void Motor_Control(void);
float Position_Control(MOTOR_REAL_INFO *MOTOR_REAL_INFO,float Target_Pos);
void Speed_Control(MOTOR_REAL_INFO* RM_MOTOR, int16_t Target_RPM);
void Homeing_Mode(MOTOR_REAL_INFO* RM_MOTOR);
void M3508_Send_Currents(void);
void VESC_Control(VESC_MOTOR_INFO *vesc);
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8]);
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR);

extern MOTOR_REAL_INFO  MOTO_REAL_INFO[8];
extern VESC_MOTOR_INFO VESC_MOTO_INFO[6];

#endif

