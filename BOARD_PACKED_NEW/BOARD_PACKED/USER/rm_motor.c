#include "rm_motor.h"

MOTOR_REAL_INFO     MOTO_REAL_INFO[8];
PID_Data MOTOR_PID_RPM[8];
PID_Data MOTOR_PID_POS[8];	//位置pid信息

/**
 * @brief 发送电机数据,使用can总线控制大疆电机。3508和2006的can id设置是一样的，而且一条can指令最多可以控制8个电机（标识符0x200: 201 202 203 204, 标识符0x1FF: 205 206 207 208）
 * @brief GM6020则是一条指令只能控制4个电机(标识符为0x1FF: 205 206 207 208， 标识符0x2FF：209 20A 20B)   
 * @brief 考虑到一条CAN能够控制的电机数量是有限的。所以该函数控制可以控制8个电机（前四个可以控制2006或者3508，后四个可以控制2006或3508或6020）
 * @param NULL
 * @return NULL
*/
void M3508_Send_Currents(void)
{
    CAN_TxHeaderTypeDef			CAN2_TxHeader_1;
	uint8_t CAN2_TxData_1[8] = {0};
	uint32_t Send_Mail_Box1;
	
	//配置控制端
	CAN2_TxHeader_1.IDE = CAN_ID_STD;
	CAN2_TxHeader_1.RTR = CAN_RTR_DATA;
	CAN2_TxHeader_1.DLC = 0x08;
	
	//配置仲裁段和数据段
	CAN2_TxHeader_1.StdId = CAN_CHASSIS_ALL_ID;//0x200
	
	CAN2_TxData_1[0] = (uint8_t)(MOTO_REAL_INFO[0].TARGET_CURRENT >> 8);	//0x201
	CAN2_TxData_1[1] = (uint8_t) MOTO_REAL_INFO[0].TARGET_CURRENT;

	CAN2_TxData_1[2] = (uint8_t)(MOTO_REAL_INFO[1].TARGET_CURRENT >> 8);	//0x202
	CAN2_TxData_1[3] = (uint8_t) MOTO_REAL_INFO[1].TARGET_CURRENT;

	CAN2_TxData_1[4] = (uint8_t)(MOTO_REAL_INFO[2].TARGET_CURRENT >> 8);	//0x203
	CAN2_TxData_1[5] = (uint8_t) MOTO_REAL_INFO[2].TARGET_CURRENT;

	CAN2_TxData_1[6] = (uint8_t)(MOTO_REAL_INFO[3].TARGET_CURRENT >> 8);	//0x204
	CAN2_TxData_1[7] = (uint8_t) MOTO_REAL_INFO[3].TARGET_CURRENT;

	if(HAL_CAN_AddTxMessage(&hcan2, &CAN2_TxHeader_1, CAN2_TxData_1, &Send_Mail_Box1)!=HAL_OK)
	{
		Error_Handler();
	}


	CAN_TxHeaderTypeDef CAN2_TxHeader_2;
	uint8_t CAN2_TxData_2[8] = {0};
	uint32_t Send_Mail_Box2;
	
	// 配置控制段
	CAN2_TxHeader_2.IDE = CAN_ID_STD;//报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	CAN2_TxHeader_2.RTR = CAN_RTR_DATA;//报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	CAN2_TxHeader_2.DLC = 0x08;//数据段长度
	CAN2_TxHeader_2.TransmitGlobalTime = DISABLE;
	
	// 配置仲裁段和数据段
	CAN2_TxHeader_2.StdId = 0x1FF;  // 用于ID为 5 6 7 8 的电机(不论是3508还是2006)

	CAN2_TxData_2[0] = (uint8_t)(MOTO_REAL_INFO[4].TARGET_CURRENT >> 8);
	CAN2_TxData_2[1] = (uint8_t) MOTO_REAL_INFO[4].TARGET_CURRENT;	       
	CAN2_TxData_2[2] = (uint8_t)(MOTO_REAL_INFO[5].TARGET_CURRENT >> 8);
	CAN2_TxData_2[3] = (uint8_t) MOTO_REAL_INFO[5].TARGET_CURRENT;	       
	CAN2_TxData_2[4] = (uint8_t)(MOTO_REAL_INFO[6].TARGET_CURRENT >> 8);
	CAN2_TxData_2[5] = (uint8_t) MOTO_REAL_INFO[6].TARGET_CURRENT;	      
	CAN2_TxData_2[6] = (uint8_t)(MOTO_REAL_INFO[7].TARGET_CURRENT >> 8);
	CAN2_TxData_2[7] = (uint8_t) MOTO_REAL_INFO[7].TARGET_CURRENT;	      
	
	
	if(HAL_CAN_AddTxMessage(&hcan2, &CAN2_TxHeader_2, CAN2_TxData_2, &Send_Mail_Box2)!=HAL_OK)
	{
		Error_Handler();
	}
	else
	{
		LED_SHOW.CAN2_Send = 1;
	}

}

/**
 * @brief 电机控制模式（GM6020待测试）
 * @param NULL
 * @return NULL
*/
void Motor_Control(void)
{
	for(int i=0; i < 8; i++)
	{
		//判断是否有选择电机类型，若无直接退出
		if (MOTO_REAL_INFO[i].Motor_Type == NONE)	
			break;
		
		//电机模式选择
		switch (MOTO_REAL_INFO[i].Motor_Mode)
		{
			case SPEED_CONTROL_MODE: //速度模式
			{
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i],MOTO_REAL_INFO[i].TARGET_RPM,MOTO_REAL_INFO[i].RPM);//速度环
				break;
            }


			case POSITION_CONTROL_MODE://位置模式
			{
				PID_Position_Calculate(&MOTOR_PID_POS[i],MOTO_REAL_INFO[i].TARGET_POS, MOTO_REAL_INFO[i].REAL_ANGLE);//位置环
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i], MOTOR_PID_POS[i].Output, MOTO_REAL_INFO[i].RPM);//速度环
				break;
			}

			case VELOCITY_PLANNING_MODE: //梯形模式
			{
				Velocity_Planning_MODE(&MOTO_REAL_INFO[i]);					
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i],MOTO_REAL_INFO[i].TARGET_RPM,MOTO_REAL_INFO[i].RPM);
				break;
			}

			case HOMEING_MODE://说实在，和速度转矩没什么差别。直接用速度转矩即可
			{
				Homeing_Mode(&MOTO_REAL_INFO[i]);	//调用校准模式
				PID_Incremental_PID_Calculation(&MOTOR_PID_RPM[i], MOTO_REAL_INFO[i].TARGET_RPM, MOTO_REAL_INFO[i].RPM); 	//速度环
				MOTOR_PID_RPM[i].Output = Max_Value_Limit(MOTOR_PID_RPM[i].Output,MOTO_REAL_INFO[i].HomingMode.TARGET_TORQUE);	//限制校准模式电流
			}

			case MOTO_OFF://电机关闭
			{
				MOTO_REAL_INFO[i].TARGET_CURRENT = 0.0f;//电流赋值
				break;
			}

			default: break;
		}
	}

	//电机转动参数
	for(int i = 0; i < 8; i++)
	{

		if(MOTO_REAL_INFO[i].Motor_Mode == CURRENT_MODE)//防止选择该模式却无法判断
		{
			
		}//电流模式下的特殊情况
		else
		{
			if(MOTO_REAL_INFO[i].Motor_Type == M_3508 && MOTO_REAL_INFO[i].Motor_Mode!=MOTO_OFF)
			{
				if(MOTO_REAL_INFO[i].CURRENT > 13000)	//额定电流 10000 * 1.3 = 13000
					MOTO_REAL_INFO[i].Motor_Mode = MOTO_OFF;
				else
					MOTO_REAL_INFO[i].TARGET_CURRENT = MOTOR_PID_RPM[i].Output*16384.0f/20000.0f;	//M3508单位毫安
			}	
			else if(MOTO_REAL_INFO[i].Motor_Type == M_2006 && MOTO_REAL_INFO[i].Motor_Mode!=MOTO_OFF)
			{
				if(MOTO_REAL_INFO[i].CURRENT > 3900)	//额定电流 3000 * 1.3 = 3900
					MOTO_REAL_INFO[i].Motor_Mode = MOTO_OFF;
				else
					MOTO_REAL_INFO[i].TARGET_CURRENT = MOTOR_PID_RPM[i].Output*10000.0f/10000.0f;	//M2006单位毫安
			}
			else if(MOTO_REAL_INFO[i].Motor_Type == M_6020 && MOTO_REAL_INFO[i].Motor_Mode!=MOTO_OFF)
			{
				if(MOTO_REAL_INFO[i].CURRENT > 2106)	//额定电流 3000 * 1.3 = 2106
					MOTO_REAL_INFO[i].Motor_Mode = MOTO_OFF;
				else
					MOTO_REAL_INFO[i].TARGET_CURRENT = MOTOR_PID_RPM[i].Output*10000.0f/10000.0f;	//(电压的给定范围是-30000 ~ 30000)
			}
			else
			{
				MOTO_REAL_INFO[i].TARGET_CURRENT = 0;
			}
		}
	}

	//使能电机，发送电流数据
	M3508_Send_Currents();
}

/**
 * @brief 最值限制函数
 * @param 传入值
 * @param 限制值(最值) 
 * @return 输出值
*/
float Max_Value_Limit(float Value, float Limit)
{
	if(Value > Limit) Value = Limit;
	if(Value < -Limit) Value = -Limit;

	return Value;
}


/**
 * @brief 速度模式
 * @param 目标转速
 * @return NULL
*/
void Speed_Control(MOTOR_REAL_INFO *RM_MOTOR, int16_t Target_RPM)
{
	RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
	RM_MOTOR->TARGET_RPM = Target_RPM;
}

/**
  * @brief  位置控制(新位置环程序)
  * @param  target_pos目标位置
  * @return 
*/
float Position_Control(MOTOR_REAL_INFO *MOTO_REAL_INFO,float target_pos)
{
	MOTO_REAL_INFO->Motor_Mode = POSITION_CONTROL_MODE;
	MOTO_REAL_INFO->TARGET_POS = target_pos;
	if(ABS(target_pos - MOTO_REAL_INFO->TARGET_POS)<1)
	{
		MOTO_REAL_INFO->TARGET_RPM = 0;
		return 1;
	}
	else
		return 0;
}

/**
  * @brief  Homing mode 回零模式
  * @param  电机结构体
  * @return NULL
 */
void Homeing_Mode(MOTOR_REAL_INFO* RM_MOTOR)
{
	float Sign_Vel = 1.0f;
	RM_MOTOR->HomingMode.flag = 0;

	if (RM_MOTOR->HomingMode.Vel >= 0)
	{
		Sign_Vel = -1.0f;
	}

	//转速赋值
	RM_MOTOR->TARGET_RPM = RM_MOTOR->HomingMode.Vel;
	
	if(                                                                                    
        fabsf(RM_MOTOR->RPM) <= 30)
	{
		RM_MOTOR->HomingMode.cnt++;
	}
	else
	{
		RM_MOTOR->HomingMode.cnt = 0;
	}
	
	if(RM_MOTOR->HomingMode.cnt >= 50) //50ms
	{
		//清除输出
		RM_MOTOR->HomingMode.cnt = 0;
		RM_MOTOR->REAL_ANGLE=0.0f;	
		RM_MOTOR->HomingMode.flag=1;
		RM_MOTOR->Motor_Mode = SPEED_CONTROL_MODE;
		RM_MOTOR->TARGET_RPM = 0;
	}
}


/**
 * @brief  电机数据读取
 * @param  can数据接收结构体
 * @param  can数据接收数组
 * @return none
*/
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8])                                    
{                                                                                         
    switch(msg -> StdId)  // 检测标准ID
	{
		case M3508_M1_ID:	//底盘3508电机1
		{ 
			// for(int i=0; i<8; i++)
				// CONTROL.CAN1_Send_Msg[0][i] = Data[i];
			
			MOTO_REAL_INFO[0].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[0].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[0].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}
		
		case M3508_M2_ID:	//底盘3508电机2
		{ 
			// for(int i=0; i<8; i++)
				// CONTROL.CAN1_Send_Msg[1][i] = Data[i];

			MOTO_REAL_INFO[1].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[1].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[1].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}
		
		case M3508_M3_ID: 	//底盘3508电机3
		{ 
			// for(int i=0; i<8; i++)
				// CONTROL.CAN1_Send_Msg[2][i] = Data[i];

			MOTO_REAL_INFO[2].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[2].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速 
			MOTO_REAL_INFO[2].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}
		
		case M3508_M4_ID:	//底盘3508电机4
		{ 
			// for(int i=0; i<8; i++)
				// CONTROL.CAN1_Send_Msg[3][i] = Data[i];

			MOTO_REAL_INFO[3].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[3].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[3].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}

		case M3508_M5_ID:
		{ 
			MOTO_REAL_INFO[4].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[4].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[4].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}

		case M3508_M6_ID:
		{ 
			MOTO_REAL_INFO[5].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[5].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[5].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}

		case M3508_M7_ID:
		{ 

			MOTO_REAL_INFO[6].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[6].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[6].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}

		case M3508_M8_ID://顶部的2006
		{
			MOTO_REAL_INFO[7].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  //转子机械角度
			MOTO_REAL_INFO[7].RPM	  = (uint16_t)((Data[2] << 8) |	 Data[3]);  //实际转子转速
			MOTO_REAL_INFO[7].CURRENT = (uint16_t)((Data[4] << 8) |	 Data[5]);  //实际转矩电流
			break;
		}
		
		default: break;
	}            
}


/**
 * @brief M3508角度积分
 * @param 电机结构体
 * @return NULL
*/
float volatile Deceleration_P = 19.0f;	//电机减速比，默认3508电机
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR)
{
	float delta_pos = 0.f;		//当前电机轴角度

	//记录第一次进入时的数据
	if(!RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
		RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}

	switch (RM_MOTOR->Motor_Type)
	{
		case M_3508:
			Deceleration_P = 19.0f;
			break;

		case M_2006:
			Deceleration_P = 36.0f;
			break;

		case M_6020:
			Deceleration_P = 1.0f;
			break;
		
		default:
			break;
	}	
	
	// 计算变化的角度
	if(RM_MOTOR->RPM == 0)
		delta_pos = 0.0f;
	
	if(RM_MOTOR->RPM > 0)
	{
		if(RM_MOTOR->ANGLE < RM_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
				delta_pos = ((float)(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8191.0f) * (360.0f/Deceleration_P);
		}
		else
		{				//这里不用滤波？？
			delta_pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8191.0f) * (360.0f/Deceleration_P);
		}
		
		// 滤波
		if(delta_pos > 0) 
			RM_MOTOR->REAL_ANGLE += delta_pos;  // 积分
	}
	else if(RM_MOTOR->RPM < 0)
	{
		if(RM_MOTOR->ANGLE > RM_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
				delta_pos = ((float)(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) / 8191.0f) * (360.0f/Deceleration_P);
		}	
		else
		{
			delta_pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8191.0f) * (360.0f/Deceleration_P);
		}
		
		// 滤波
		if(delta_pos < 0) 
			RM_MOTOR->REAL_ANGLE += delta_pos;  // 积分
	}

	// 存储角度值
	RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
}



/**
 * @brief 梯度速度规划
 * @param M电机结构体
 * @return NULL
*/
void Velocity_Planning_MODE(MOTOR_REAL_INFO *M3508_MOTOR)	
{
	//static int cnt;//记时用
	float Ssu;   //总路程
	float Sac;   //加速路程
	float Sde;   //减速路程
	float Sco;   //匀速路程
	float Aac;   //加速加速度
	float Ade;   //减速加速度
	float S;     //当前路程
	
	// 如果所配数据有误，则不执行速度规划		
	if((M3508_MOTOR->Velocity_Planning.Rac > 1) || (M3508_MOTOR->Velocity_Planning.Rac < 0) ||		//加速路程的比例
		 (M3508_MOTOR->Velocity_Planning.Rde > 1) || (M3508_MOTOR->Velocity_Planning.Rde < 0) ||	//减速路程的比例
		 (M3508_MOTOR->Velocity_Planning.Vmax < M3508_MOTOR->Velocity_Planning.Vstart) )			//最大的速度<开始的速度 
	{
		M3508_MOTOR->TARGET_RPM = 0;  // 令夹爪不运动
		return;
	}
	// 匀速模式
	if(M3508_MOTOR->Velocity_Planning.Pstart == M3508_MOTOR->Velocity_Planning.Pend)	//开始位置=结束位置
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vmax;	//开始的速度*最大的速度
		return;
	}
	
	// 计算一些变量
	Ssu = ABS(M3508_MOTOR->Velocity_Planning.Pend - M3508_MOTOR->Velocity_Planning.Pstart); 	//总路程   
	Sac = Ssu * M3508_MOTOR->Velocity_Planning.Rac;		//加速路程 =	总路程 * 加速路程的比例
	Sde = Ssu * M3508_MOTOR->Velocity_Planning.Rde;		//减速路程 =	总路程 * 减速路程的比例
	Sco = Ssu - Sac - Sde;								//匀速路程 = 总路程 - 加速路程 - 减速路程
	Aac = (M3508_MOTOR->Velocity_Planning.Vmax * M3508_MOTOR->Velocity_Planning.Vmax - M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart) / (2.0f * Sac);	//加速加速度 (最大的速度*最大的速度 - 开始的速度 *开始的速度 ) / (2.0f * 加速路程)
	Ade = (M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend -   M3508_MOTOR->Velocity_Planning.Vmax *   M3508_MOTOR->Velocity_Planning.Vmax) / (2.0f * Sde);	  
	
	// 过滤异常情况
	if(((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pstart)) ||		//[(结束位置 > 开始位置) && (处理过的真实角度pos <开始位置)]	||
		 ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pstart)))		//	[(结束位置 < 开始位置) && (处理过的真实角度pos >开始位置)]
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart;	//TARGET_RPM = 开始的速度
	}
	else if(((M3508_MOTOR->Velocity_Planning.Pend > M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE > M3508_MOTOR->Velocity_Planning.Pend)) ||
		      ((M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) && (M3508_MOTOR->REAL_ANGLE < M3508_MOTOR->Velocity_Planning.Pend)))
	{
		M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vstart;	//TARGET_RPM = 末尾的速度
	}
	else
	{
		S = ABS(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pstart);      //开始位置
		
		// 规划RPM
		if     (S < Sac)       M3508_MOTOR->TARGET_RPM = sqrt(2.0f * Aac * S + M3508_MOTOR->Velocity_Planning.Vstart * M3508_MOTOR->Velocity_Planning.Vstart);               // 加速阶段
		else if(S < (Sac+Sco)) M3508_MOTOR->TARGET_RPM = M3508_MOTOR->Velocity_Planning.Vmax;                                                        // 匀速阶段
		else                   M3508_MOTOR->TARGET_RPM = sqrt(M3508_MOTOR->Velocity_Planning.Vend * M3508_MOTOR->Velocity_Planning.Vend - 2.0f * Ade * ABS(Ssu - S));  // 减速阶段
	}
	 
	// 分配合适的正负号
	if(M3508_MOTOR->Velocity_Planning.Pend < M3508_MOTOR->Velocity_Planning.Pstart) 
		M3508_MOTOR->TARGET_RPM = -M3508_MOTOR->TARGET_RPM;
	//判断是否完成
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) < 3)
	{
		M3508_MOTOR->Velocity_Planning.flag = 1;//设置标志位
		M3508_MOTOR->TARGET_RPM=0;
	}
		
						
	if((fabsf(M3508_MOTOR->REAL_ANGLE - M3508_MOTOR->Velocity_Planning.Pend)) > 3)
	{
		M3508_MOTOR->Velocity_Planning.flag = 0;
	}
}


