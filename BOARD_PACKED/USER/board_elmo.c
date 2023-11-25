#include "board_elmo.h"
uint16_t test_con_id;


RM_BRD_MSG RM_BOARD_MSG_1[8];
RM_BRD_MSG RM_BOARD_MSG_2[8];
VESC_BRD_MSG VESC_BOARD_MSG_1[6];
VESC_BRD_MSG VESC_BOARD_MSG_2[6];


/**
 * @brief 大疆电机类型配置，一定要配置，否则电机将不会运行
 * @param NUM BOARD1 or ... BOARD4
 * @param MODE MODE1~MODE8 电机类型。M_3508 or M_2006 or M_6020 or NONE(have no motor)
*/
void BOARD_RM_MOTOR_INIT(BOARD_NUM NUM, MotorType_TypeDef MODE1,MotorType_TypeDef MODE2,MotorType_TypeDef MODE3,
                                        MotorType_TypeDef MODE4,MotorType_TypeDef MODE5,MotorType_TypeDef MODE6,
                                        MotorType_TypeDef MODE7,MotorType_TypeDef MODE8)
{
    uint8_t TxData[8];
    int16_t controller_id = 1;
    
    TxData[0] = MODE1;
    TxData[1] = MODE2;
    TxData[2] = MODE3;
    TxData[3] = MODE4;
    TxData[4] = MODE5;
    TxData[5] = MODE6;
    TxData[6] = MODE7;
    TxData[7] = MODE8;

    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|MOTOR_TYPE_CONFIG,TxData,0x08);
}


/**
 * @brief 大疆电机速度控制模式
 * @param NUM BOARD1 or ... BOARD4
 * @param target_rpm 目标转速，单位为erpm
 * @param controller_id 电机ID,大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
void BOARD_RM_SPEED_CONTROL(BOARD_NUM NUM, uint16_t controller_id, int16_t target_rpm)
{
    int index = 0;
    uint8_t TxData[8];
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_int32(TxData,target_rpm,&index);
    buffer_append_int32(TxData,0,&index);

    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|SPEED_CONTROL_MODE,TxData,index);
}


/**
 * @brief 大疆电机位置控制模式
 * @param NUM BOARD1 or ... BOARD4
 * @param target_pos 目标位置，单位为度
 * @param controller_id 电机ID,大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
void BOARD_RM_POSITION_CONTROL(BOARD_NUM NUM, uint16_t controller_id, int16_t target_pos)
{
    int index = 0;
    uint8_t TxData[8];
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_int32(TxData,target_pos,&index);
    buffer_append_int32(TxData,0,&index);

    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|POSITION_CONTROL_MODE,TxData,index);
}


/**
 * @brief 大疆电机回零校准模式
 * @param NUM BOARD1 or ... BOARD4
 * @param target_rpm 目标转速
 * @param target_tarque 最大电流限制，即限制力矩
 * @param controller_id 电机ID,大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
void BOARD_RM_HOMEING_MODE(BOARD_NUM NUM, uint16_t controller_id, int16_t target_rpm, int16_t target_tarque)
{
    int index = 0;
    uint8_t TxData[8];
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_int32(TxData,target_rpm,&index);
    buffer_append_int32(TxData,target_tarque,&index);

    comm_can_transmit_stdid(&hcan1, (controller_id<<6)|HOMEING_MODE,TxData,index);
}


/**
 * @brief 大疆电机速度环PID配置函数，只需要在最开头执行几次即可
 * @param NUM BOARD1 or ... BOARD4
 * @param controller_id 电机ID,大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
void BOARD_RM_VEL_PID(BOARD_NUM NUM, uint16_t controller_id, float Kp, float Ki, uint16_t Out_Max, int16_t Dead_Size)
{
    int index = 0;
    uint8_t TxData[10];
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_float16(TxData,Kp,1e3,&index,1);
    buffer_append_float16(TxData,Ki,1e3,&index,1);
    buffer_append_uint16(TxData,Out_Max,&index);
    buffer_append_int16(TxData,Dead_Size,&index);

    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|VEL_PID_CONFIG,TxData,index);
}


/**
 * @brief 大疆电机位置环PID配置函数，只需要在最开头执行几次即可
 * @param NUM BOARD1 or ... BOARD4
 * @param Dead_Size 死区设置，为负数时不开启死区
 * @param controller_id 电机ID,大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
void BOARD_RM_POS_PID(BOARD_NUM NUM, uint16_t controller_id, float Kp, float Kd, uint16_t Out_Max, int16_t Dead_Size)
{
    int index = 0;
    uint8_t TxData[8];
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_float16(TxData,Kp,1e2,&index,1);
    buffer_append_float16(TxData,Kd,1e2,&index,1);
    buffer_append_uint16(TxData,Out_Max,&index);
    buffer_append_int16(TxData,Dead_Size,&index);

    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|POS_PID_CONFIG,TxData,index);
}


/**
 * @brief 设置速度规划的参数，开启速度规划控制
 * @param NUM 板子ID
 * @param controller_id 电机id
 * @param Pstart 开始位置
 * @param Pend 结束位置
 * @param Vstart 开始的速度  单位：RPM 绝对值
 * @param Vmax 最大的速度
 * @param Vend 末尾的速度
 * @param Rac 加速路程的比例
 * @param Rde 减速路程的比例
 * @retval NULL
*/
void BOARD_RM_VEL_TPLAN(BOARD_NUM NUM, uint16_t controller_id, int16_t Pstart, int16_t Pend, uint16_t Vstart, uint16_t Vmax, uint16_t Vend, float Rac, float Rde)
{
    static int16_t send_index = 0;
    int index = 0;
    uint8_t TxData[8];
    controller_id = board_id_choose(NUM,controller_id);

    if(send_index==0)
    {
        buffer_append_int16(TxData,Pstart,&index);
        buffer_append_int16(TxData,Pend,&index);
        buffer_append_uint16(TxData,Vstart,&index);
        buffer_append_uint16(TxData,Vmax,&index);

        comm_can_transmit_stdid(&hcan1,(controller_id<<6)|VELOCITY_PLANNING_MODE,TxData,index);
    }
    
    if(send_index == 2)
    {
        buffer_append_uint16(TxData,Vend,&index);
        buffer_append_float16(TxData,Rac,1e3,&index,1);
        buffer_append_float16(TxData,Rde,1e3,&index,1);
        buffer_append_int16(TxData,0,&index);

        comm_can_transmit_stdid(&hcan1,(controller_id<<6)|VELOCITY_PLANNING_MODE_CFG2,TxData,index);
    }

    send_index++;
    if(send_index>2)
        send_index = 0;
}


/**
 * @brief 电机使能，速度环模式且赋值RPM=0
 * @param NUM BOARD1 or ... BOARD4
 * @param controller_id 电机ID,大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
void BOARD_RM_MOTOR_ON(BOARD_NUM NUM, uint16_t controller_id)
{
    uint8_t TxData[1] = {0};
    controller_id = board_id_choose(NUM,controller_id);

    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|MOTOR_ON,TxData,0x01);
}


/**
 * @brief 电机失能
 * @param NUM BOARD1 or ... BOARD4
 * @param controller_id 电机ID,大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
void BOARD_RM_MOTOR_OFF(BOARD_NUM NUM, uint16_t controller_id)
{
    uint8_t TxData[1] = {0};
    controller_id = board_id_choose(NUM,controller_id);

    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|MOTO_OFF,TxData,0x01);
}


/**
 * @brief 电机分控板，VESC电机电转速设置函数
 * @param NUM 电机板ID
 * @param target_rpm 单位是电转速/eRPM, eRPM = 电机的极对数*机械转速
 * @param controller_id 电机id号
*/
void BOARD_VESC_SPEED_CONTROL(BOARD_NUM NUM, uint16_t controller_id, int32_t target_rpm)
{
    uint8_t TxData[4];
    int index = 0;
    test_con_id = board_id_choose(NUM,controller_id);

    buffer_append_int32(TxData,target_rpm,&index);
    comm_can_transmit_stdid(&hcan1,(test_con_id<<6)|VESC_SPEED,TxData,index);
}


/**
 * @brief 电机分控板，VESC电机占空比设置函数
 * @param NUM 电机板ID
 * @param duty 占空比 0.24 = 24%
 * @param controller_id 电机id号
*/
void BOARD_VESC_SET_DUTY(BOARD_NUM NUM, uint16_t controller_id, float duty)
{
    uint8_t TxData[4];
    int index = 0;
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_uint32(TxData,(uint32_t)(duty * 100000),&index);
    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|VESC_DUTY,TxData,index);
}


/**
 * @brief 电机分控板，VESC电机电流设置函数
 * @param NUM 电机板ID
 * @param current 电流 单位A
 * @param controller_id 电机id号
*/
void BOARD_VESC_SET_CURRENT(BOARD_NUM NUM, uint16_t controller_id, float current)
{
    uint8_t TxData[4];
    int index = 0;
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_int32(TxData, (int32_t)(current * 1000), &index);
    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|VESC_CURRENT,TxData,index);
}


/**
 * @brief 电机分控板，VESC电机位置模式函数，极少用（N5065使用pos有些问题，不要尝试）
 * @param NUM 电机板ID
 * @param pos 角度
 * @param controller_id 电机id号
*/
void BOARD_VESC_SET_POS(BOARD_NUM NUM, uint16_t controller_id, float pos)
{
    uint8_t TxData[4];
    int index = 0;
    controller_id = board_id_choose(NUM,controller_id);

    buffer_append_int32(TxData, (int32_t)(pos * 1000000), &index);
    comm_can_transmit_stdid(&hcan1,(controller_id<<6)|VESC_POS,TxData,index);
}


//测试函数
void RM_SPPED_TEST(void)
{
    static int16_t index=0;
    for(int16_t i=0; i<2; i++)
    {
        rm_motor_control(&RM_BOARD_MSG_1[index+i],BOARD_RM1);
    }
    
    index+=2;
    if(index >= 4)
        index = 0;
}


/**
 * @brief 从分控板上获取电机数据
*/
void get_rm_board_msg(CAN_RxHeaderTypeDef *CAN, uint8_t* RxData)
{
    int index = 0;
    uint16_t board_id = (CAN->StdId >> 8);
    uint16_t id = CAN->StdId & 0xFF;

    switch (board_id)
    {
        case BOARD_RM1:
        {
            RM_BOARD_MSG_1[id-1].REAL_ANGLE = buffer_get_float32(RxData,1e3,&index,0);
            RM_BOARD_MSG_1[id-1].RPM = buffer_get_int16(RxData,&index);
            RM_BOARD_MSG_1[id-1].CURRETN = buffer_get_int16(RxData,&index);
            break;
        }

        case BOARD_RM2:
        {
            RM_BOARD_MSG_2[id-1].REAL_ANGLE = buffer_get_float32(RxData,1e3,&index,0);
            RM_BOARD_MSG_2[id-1].RPM = buffer_get_int16(RxData,&index);
            RM_BOARD_MSG_2[id-1].CURRETN = buffer_get_int16(RxData,&index);
            break;
        }
        
        case BOARD_VESC1:
        {
            VESC_BOARD_MSG_1[id-1].RPM = buffer_get_float16(RxData,1,&index,0);
            VESC_BOARD_MSG_1[id-1].CURRETN = buffer_get_float16(RxData,1e3,&index,0);
            VESC_BOARD_MSG_1[id-1].DUTY = buffer_get_float16(RxData,1e2,&index,1);
            VESC_BOARD_MSG_1[id-1].POS = buffer_get_float16(RxData,1,&index,0);
        }

        case BOARD_VESC2:
        {
            VESC_BOARD_MSG_2[id-1].RPM = buffer_get_float16(RxData,1,&index,0);
            VESC_BOARD_MSG_2[id-1].CURRETN = buffer_get_float16(RxData,1e3,&index,0);
            VESC_BOARD_MSG_2[id-1].DUTY = buffer_get_float16(RxData,1e2,&index,1);
            VESC_BOARD_MSG_2[id-1].POS = buffer_get_float16(RxData,1,&index,0);
        }

        default:
            break;
    }
    
}


/**
 * @brief 发送端控制id选择函数。1到8是对应1号板(大疆电机板)，9到16对应2号板(大疆电机板)。17到22对应3号板(VESC电机板)，23到28对应4号板(VESC电机板)
 * @param NUM BOARD1 or ... BOARD4
 * @param controller_id 电机ID 大疆电调闪几下就是多少(闪一下就是1)。VESC的需要在上位机进行设置，比较复杂，但本固件都写死，1~6
*/
int16_t board_id_choose(BOARD_NUM NUM, uint16_t controller_id)
{
    int16_t id;
    switch (NUM)
    {
        case BOARD_RM1:     //一个大疆电机板控制8个电机
            id = controller_id;
            break;

        case BOARD_RM2:
        {
            id = controller_id + 8;
            break;
        }
        
        case BOARD_VESC1:   //一个VESC控制板控制6个无刷电机
        {
            if(controller_id>=1&&controller_id<=6)
                id = controller_id + 16;
            else
                break;
            break;
        }
        
        case BOARD_VESC2:
        {
            if(controller_id>=1&&controller_id<=6)
                id = controller_id + 22;
            else
                break;
            break;
        }

        default: break;
    }

    return id;
}

