#ifndef  __CHASSIS_H
#define  __CHASSIS_H

void Robot_Wheels_Adjust(void);

#define COS60 0.500000f
#define COS30 0.866025f
#define COS45 0.707106f
#define V_REAL    0.128f/60                      //轮子的线速度
#define  PI              3.1415926f    //PI的值
#define WHEEL_R              0.152f/2                 //轮子半径 
#define RM_transition_MS     (PI*WHEEL_R)/570.0f      //转速与速度的转换  
#define MS_transition_RM     570.0f/(PI*WHEEL_R)      //速度与转速的转换    
#define CHASSIS_R 0.374f

#define Mecanum_Rx 0.5
#define Mecanum_Ry 0.5


//机器人底盘数据结构体
typedef struct ROBOT_SPEED
{
 	float World_W;
 	float Robot_VX;
 	float Robot_VY;
} ROBOT_SPEED;


//世界坐标系结构体
typedef struct WORLD_VEL
{
	float World_X;
	float World_Y;
	float World_W;
}WORLD_VEL;

typedef struct ROBOT_POSITION
{
	float POSITION_X;
	float POSITION_Y;
}ROBOT_POSITION;

typedef struct SPEED_LIMIT
{
	float Vy_MAX;
	float Vx_MAX;
	float Vw_MAX;
}SPEED_LIMIT;

typedef struct ROBOT_CHASSIS
{
	ROBOT_SPEED SPEED;
	WORLD_VEL WORLD;
	ROBOT_POSITION POSITION;
	float Motor_Target_RPM[4];           //4个轮子的目标转速
	int8_t Chassis_Controller_Flag;
	int8_t World_Move_Flag;
	float Expect_Angle ;
	float Angle;
	SPEED_LIMIT SPEED_LIMI;
} ROBOT_CHASSIS;


extern ROBOT_CHASSIS ROBOT_CHASSI;
#endif

