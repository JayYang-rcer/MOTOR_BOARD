#include "control.h"

CONTROL_S CONTROL;

/***********************************************************************************************************************************************************************
CAN帧的ID可以分解为两个段，前5位作为驱动器ID，后6位作为电机命令(电机开、电机关、速度环PID设置、位置环
PID设置、电机类型确认、速度模式、位置模式、力矩回零模式)
驱动器ID为0~127，分别对应到大疆电机结构体中
电机工作模式固定为宏定义，请勿更改
***********************************************************************************************************************************************************************/
int16_t controller_id;  //电机控制id
int16_t control_mode;   //控制模式储存变量
int16_t n;  //用来确定对哪个电机赋值    
int16_t MOTOR_TYPE; //电机类型
void rm_motor_control_comm(CAN_RxHeaderTypeDef* CAN, uint8_t* RxData)
{
    static int8_t vel_plan_flag = 0;
    controller_id = ((CAN->StdId >> 6) & 0x1F);
    control_mode = (CAN->StdId & 0x3F);
    
    switch (CONTROL.BOARD_ID)   //选定电机板ID，在电机板上使用拨码设置
    {
        case BOARD_RM1:
            n = controller_id - 1;
            break;

        case BOARD_RM2:
            n = controller_id - 9;
            break;

        case BOARD_VESC1:
            n = controller_id - 17;
            break;

        case BOARD_VESC2:
            n = controller_id - 25;
            break;
        
        default: 
            break;
    }

    if(control_mode != HOMEING_MODE)
        MOTO_REAL_INFO[n].HomingMode.flag = 0;

    if((control_mode>3&&control_mode<17) && (n>=0&&n<=7))
    {
        switch(control_mode)
        {
            case SPEED_CONTROL_MODE:    //速度控制模式
            {
                int index = 0;  //记录数组操作到哪个位置
                CONTROL.MOTOR_TAR_VALUE[n] = buffer_get_int32(RxData,&index);
                CONTROL.MOTOR_TARQUE[n] = buffer_get_int32(RxData,&index);
                MOTO_REAL_INFO[n].Motor_Mode = SPEED_CONTROL_MODE;
                MOTO_REAL_INFO[n].TARGET_RPM = CONTROL.MOTOR_TAR_VALUE[n];
                break;
            }
            
            case POSITION_CONTROL_MODE:     //位置控制模式
            {
                int index = 0;  //记录数组操作到哪个位置
                CONTROL.MOTOR_TAR_VALUE[n] = buffer_get_int32(RxData,&index);
                CONTROL.MOTOR_TARQUE[n] = buffer_get_int32(RxData,&index);
                Position_Control(&MOTO_REAL_INFO[n],CONTROL.MOTOR_TAR_VALUE[n]);
                break;
            }

            case HOMEING_MODE:      //回零校准模式
            {
                int index = 0;  //记录数组操作到哪个位置
                static int32_t last_value;
                CONTROL.MOTOR_TAR_VALUE[n] = buffer_get_int32(RxData,&index);
                CONTROL.MOTOR_TARQUE[n] = buffer_get_int32(RxData,&index);

                if(last_value != CONTROL.MOTOR_TAR_VALUE[n])    //上个目标值与当前目标值是否相同
                {
                    MOTO_REAL_INFO[n].HomingMode.flag = 0;
                }
                if(MOTO_REAL_INFO[n].HomingMode.flag == 0)
                {
                    MOTO_REAL_INFO[n].Motor_Mode = HOMEING_MODE;
                }
                else
                {
                    MOTO_REAL_INFO[n].Motor_Mode = SPEED_CONTROL_MODE;
                    MOTO_REAL_INFO[n].TARGET_RPM = 0;
                }

                MOTO_REAL_INFO[n].HomingMode.Vel= CONTROL.MOTOR_TAR_VALUE[n];
                MOTO_REAL_INFO[n].HomingMode.TARGET_TORQUE = CONTROL.MOTOR_TARQUE[n];

                last_value = CONTROL.MOTOR_TAR_VALUE[n];
                break;
            }

            case VELOCITY_PLANNING_MODE:    //大疆电机梯形速度规划
            {   
                int index = 0;
                static int32_t last_Pstart;
                MOTO_REAL_INFO[n].Velocity_Planning.Pstart = buffer_get_int16(RxData,&index);
                MOTO_REAL_INFO[n].Velocity_Planning.Pend = buffer_get_int16(RxData,&index);
                MOTO_REAL_INFO[n].Velocity_Planning.Vstart = buffer_get_uint16(RxData,&index);
                MOTO_REAL_INFO[n].Velocity_Planning.Vmax = buffer_get_uint16(RxData,&index);

                //判断是否再次使用梯形规划(例如机构抬升)
                if(last_Pstart != MOTO_REAL_INFO[n].Velocity_Planning.Pstart)
                    MOTO_REAL_INFO[n].Velocity_Planning.flag=0;

                //由于梯形速度规划的参数太多，故分为两帧发送，需要集齐所有参数才可以触发速度规划模式
                if(vel_plan_flag == 1 && MOTO_REAL_INFO[n].Velocity_Planning.flag!=1)
                {
                    MOTO_REAL_INFO[n].Motor_Mode = VELOCITY_PLANNING_MODE;
                    vel_plan_flag = 0;
                }
                else
                {
                    //防止出现意外致使电机不停地转动
                    MOTO_REAL_INFO[n].Motor_Mode = SPEED_CONTROL_MODE;
                    MOTO_REAL_INFO[n].TARGET_RPM = 0;
                }
                
                //记录上一次的启动位置
                last_Pstart = MOTO_REAL_INFO[n].Velocity_Planning.Pstart;
                break;
            }

            //梯形速度规划的第二帧参数
            case VELOCITY_PLANNING_MODE_CFG2:
            {
                int index = 0;
                MOTO_REAL_INFO[n].Velocity_Planning.Vend = buffer_get_uint16(RxData,&index);
                MOTO_REAL_INFO[n].Velocity_Planning.Rac = buffer_get_float16(RxData,1e3,&index,1);
                MOTO_REAL_INFO[n].Velocity_Planning.Rde = buffer_get_float16(RxData,1e3,&index,1);
                vel_plan_flag = 1;
                break;
            }

            case MOTO_OFF:      //电机失能
            {
                MOTO_REAL_INFO[n].Motor_Mode = MOTO_OFF;
                break;
            }

            case MOTOR_ON:      //电机使能
            {
                Speed_Control(&MOTO_REAL_INFO[n],0);
                break;
            }
            
            case VEL_PID_CONFIG:    //速度环PID设置
            {
                int index = 0;  //记录数组操作到哪个位置
                MOTOR_PID_RPM[n].K_P = buffer_get_float16(RxData,1e3,&index,1);
                MOTOR_PID_RPM[n].K_I = buffer_get_float16(RxData,1e3,&index,1);
                MOTOR_PID_RPM[n].Out_MAX = buffer_get_uint16(RxData,&index);
                MOTOR_PID_RPM[n].Dead_Size = buffer_get_int16(RxData,&index);
                break;
            }

            case POS_PID_CONFIG:    //位置环PID设置
            {
                int index = 0;  //记录数组操作到哪个位置
                MOTOR_PID_POS[n].K_P = buffer_get_float16(RxData,1e2,&index,1);
                MOTOR_PID_POS[n].K_D = buffer_get_float16(RxData,1e2,&index,1);
                MOTOR_PID_POS[n].Out_MAX = buffer_get_uint16(RxData,&index);
                MOTOR_PID_POS[n].Dead_Size = buffer_get_int16(RxData,&index);
                break;
            }

            case MOTOR_TYPE_INIT:   //电机类型初始化
            {
                //直接使用一个数组，真香
                for(int i=0; i<8; i++)
                {
                    MOTO_REAL_INFO[i].Motor_Type = RxData[i];
                }
            }

            default:
                break;
        }
    }
    else
    {
        // MOTO_REAL_INFO[n].TARGET_CURRENT = 0;
        MOTO_REAL_INFO[n].Motor_Mode = MOTO_OFF;
    }
}


/**
 * @brief vesc驱动器控制函数，接收来自主板的can信号控制vesc驱动器
*/
void vesc_motor_control_comm(CAN_RxHeaderTypeDef* CAN, uint8_t* RxData)
{
    controller_id = ((CAN->StdId >> 6) & 0x1F);
    control_mode = (CAN->StdId & 0x3F);
	
	switch (CONTROL.BOARD_ID)   //选定电机板ID，在电机板上使用拨码设置
    {
        case BOARD_RM1:
            n = controller_id - 1;
            break;

        case BOARD_RM2:
            n = controller_id - 9;
            break;

        case BOARD_VESC1:
            n = controller_id - 17;
            break;

        case BOARD_VESC2:
            n = controller_id - 25;
            break;
        
        default: break;
    }
	
	
    if((control_mode>=0&&control_mode<=3) && (n>=0&&n<=5))
    {
        switch (control_mode)
        {

            case VESC_CURRENT:  //电流模式
            {
                int index = 0;  //记录数组操作到哪个位置
                VESC_MOTO_INFO[n].MOTOR_MODE = VESC_CURRENT;
                VESC_MOTO_INFO[n].TARGET_CURRENT = ((float)buffer_get_int32(RxData,&index))/1000;
                break;
            }

            case VESC_DUTY:     //占空比模式
            {
                int index = 0;  
                VESC_MOTO_INFO[n].MOTOR_MODE = VESC_DUTY;
                VESC_MOTO_INFO[n].TARGET_VESC_DUTY = ((float)buffer_get_uint32(RxData,&index))/100000;
                break;
            }

            case VESC_SPEED:    //速度模式
            {
                int index = 0;
                VESC_MOTO_INFO[n].MOTOR_MODE = VESC_SPEED;
                VESC_MOTO_INFO[n].TARGET_eRPM = buffer_get_int32(RxData,&index);
                break;
            }

            case VESC_POS:      //位置模式，一般不用
            {
                int index = 0;
                VESC_MOTO_INFO[n].MOTOR_MODE = VESC_POS;
                VESC_MOTO_INFO[n].TARGET_ANGLE = ((float)buffer_get_int32(RxData,&index))/1000000;
                break;
            }
            
            default: 
                break;
        }
    }
}


/**
 * @brief 主板can信号丢失处理函数，假如can信号断开，便会失能电机
*/
void can_loss(void)
{
    if(CONTROL.CAN1_RECIEVE_FLAG==1 && CONTROL.CAN_TIMER==1)
    {
        if(CONTROL.BOARD_ID==BOARD_RM1 || CONTROL.BOARD_ID==BOARD_RM2)
        {
            for(int i=0; i<8; i++)
            {
            MOTO_REAL_INFO[i].Motor_Mode = MOTO_OFF;
            }
        }
        else
        {
            for(int i=0; i<5; i++)
            {
                VESC_MOTO_INFO[i].MOTOR_MODE = VESC_CURRENT;
                VESC_MOTO_INFO[i].TARGET_CURRENT = 0;
            }
        }
    }
}


/**
 * @brief 向主控板发送大疆电机的实时数据，100ms发送一次
*/
void Send_Rm_Motor_Msg(void)
{   
    uint8_t CAN1_TxData[8]={0};
    static int can_index=0; //用来缓冲can数据的发送，防止炸can

    for(int i=0; i<2; i++)
    {
        if(i+can_index < 8)
        {
            int index = 0;
            uint16_t send_id;
            buffer_append_float32(CAN1_TxData,MOTO_REAL_INFO[i+can_index].REAL_ANGLE,1e3,&index, 0);
            buffer_append_int16(CAN1_TxData,MOTO_REAL_INFO[i+can_index].RPM,&index);
            buffer_append_int16(CAN1_TxData,MOTO_REAL_INFO[i+can_index].CURRENT,&index);
            
            if(CONTROL.BOARD_ID == BOARD_RM1)
            {
                send_id = (BOARD_RM1 << 8)|(i+can_index+1);
                comm_can_transmit_stdid(&hcan1,send_id,CAN1_TxData,0x08);
                LED_SHOW.CAN1_Send = 1;
            }

            if(CONTROL.BOARD_ID == BOARD_RM2)
            {
                send_id = (BOARD_RM2 << 8)|(i+can_index+1);
                comm_can_transmit_stdid(&hcan1,send_id,CAN1_TxData,0x08);
                LED_SHOW.CAN1_Send = 1;
            }
        }
    }
    can_index+=2;
    if(can_index>=8)
        can_index = 0;
}   


/**
 * @brief 向主控板发送VESC驱动器的实时数据，100ms发送一次
*/
void Send_Vesc_Motor_Msg(void)
{
    uint8_t CAN1_TxData[8]={0};
    static int can_index=0; //用来缓冲can数据的发送，防止炸can

    for(int i=0; i<2; i++)
    {
        if(i+can_index < 6)
        {
            int index = 0;
            uint16_t send_id;
            buffer_append_float16(CAN1_TxData,VESC_MOTO_INFO[i+can_index].NOW_eRPM,1,&index, 0);
            buffer_append_float16(CAN1_TxData,VESC_MOTO_INFO[i+can_index].NOW_CURRENT,1e3,&index,0);
            buffer_append_float16(CAN1_TxData,VESC_MOTO_INFO[i+can_index].NOW_VESC_DUTY,1e2,&index, 1);
            buffer_append_float16(CAN1_TxData,VESC_MOTO_INFO[i+can_index].NOW_ANGLE,1,&index, 0);

            if(CONTROL.BOARD_ID == BOARD_VESC1)
            {
                send_id = (BOARD_VESC1 << 8)|(i+can_index+1);
                comm_can_transmit_stdid(&hcan1,send_id,CAN1_TxData,0x08);
                LED_SHOW.CAN1_Send = 1;
            }
            
            if(CONTROL.BOARD_ID == BOARD_VESC2)
            {
                send_id = (BOARD_VESC2 << 8)|(i+can_index+1);
                comm_can_transmit_stdid(&hcan1,send_id,CAN1_TxData,0x08);
                LED_SHOW.CAN1_Send = 1;
            }
        }
    }
}

