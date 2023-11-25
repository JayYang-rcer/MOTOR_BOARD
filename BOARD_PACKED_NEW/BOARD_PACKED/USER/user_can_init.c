#include "user_can_init.h"
/*****************************************************************************************
使用CAN1与主控板进行通讯
使用CAN2与大疆电机进行通讯
*****************************************************************************************/


/**
 * @brief 配置CAN滤波器，开启CAN1
*/
void User_Open_Can1(void)
{
	CAN_FilterTypeDef Filter;
	
	Filter.FilterBank = 0;	//过滤器组别0
    Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    Filter.FilterScale = CAN_FILTERSCALE_32BIT;
    Filter.FilterIdHigh         = (((uint32_t)CAN_RxExtId<<3)&0xffff0000)>>16;				/* 要过滤的ID高位 *///0x0000
  	Filter.FilterIdLow          = (((uint32_t)CAN_RxExtId<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff0000; /* 要过滤的ID低位 *///0x0000	
    Filter.FilterMaskIdHigh = 0x0000;	//拓展帧标识符
    Filter.FilterMaskIdLow = 0x0000;
    Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
	Filter.FilterActivation = ENABLE;	//使能过滤器
	Filter.SlaveStartFilterBank = 14;
	
	if (HAL_CAN_ConfigFilter(&hcan1, &Filter) != HAL_OK)
	{
			/* Filter configuration Error */
			Error_Handler();
	}
		
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
		
		/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}


/**
 * @brief 开启CAN2
*/
void User_Open_Can2(void)
{
    CAN_FilterTypeDef Filter;

    if(CONTROL.BOARD_ID==BOARD_RM1 || CONTROL.BOARD_ID==BOARD_RM2)
    {
        Filter.FilterBank = 0;	//过滤器组别0
        Filter.FilterMode = CAN_FILTERMODE_IDMASK;
        Filter.FilterScale = CAN_FILTERSCALE_32BIT;
        Filter.FilterIdHigh         = 0;
        Filter.FilterIdLow          = 0;
        Filter.FilterMaskIdHigh = 0x0000;	//拓展帧标识符
        Filter.FilterMaskIdLow = 0x0000;
        Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        Filter.FilterActivation = ENABLE;	//使能过滤器
        Filter.SlaveStartFilterBank = 0;
    }
    else
    {
        Filter.FilterBank = 15;	//过滤器组别15
        Filter.FilterMode = CAN_FILTERMODE_IDMASK;
        Filter.FilterScale = CAN_FILTERSCALE_32BIT;
        Filter.FilterIdHigh         = (((uint32_t)CAN_RxExtId<<3)&0xffff0000)>>16;				/* 要过滤的ID高位 *///0x0000
        Filter.FilterIdLow          = (((uint32_t)CAN_RxExtId<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xffff0000; /* 要过滤的ID低位 *///0x0000	
        Filter.FilterMaskIdHigh = 0x0000;	//拓展帧标识符
        Filter.FilterMaskIdLow = 0x0000;
        Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
        Filter.FilterActivation = ENABLE;	//使能过滤器
        Filter.SlaveStartFilterBank = 14;
    }

	
	if (HAL_CAN_ConfigFilter(&hcan2, &Filter) != HAL_OK)
	{
			/* Filter configuration Error */
			Error_Handler();
	}
		
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
		
		/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
	
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}


uint8_t CAN1_RxData[8];
uint8_t CAN2_RxData[8];
uint8_t test_id = 0;

/**
 * @brief hal库CAN回调函数,接收电机数据
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(hcan == &hcan1)
    {
        CONTROL.CAN1_RECIEVE_FLAG = 1;
        CONTROL.CAN_TIMER = 0;  //接收到消息时重置定时器超时标志位
        LED_SHOW.CAN1_Recieve = 1;
        CAN_RxHeaderTypeDef CAN1_RX0_Message;

        //从主控板获取命令can数据
        if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_RX0_Message,CAN1_RxData) != HAL_OK)
        {
            Error_Handler();
        }

        if(CAN1_RX0_Message.IDE == CAN_ID_STD)  //标准帧
        {
            if(CONTROL.BOARD_ID==1 || CONTROL.BOARD_ID==2)      //判断分控板ID来使用哪一套固件
                rm_motor_control_comm(&CAN1_RX0_Message,CAN1_RxData);
            else
                vesc_motor_control_comm(&CAN1_RX0_Message,CAN1_RxData);
        }  
    }

    if(hcan == &hcan2)
    {
        CAN_RxHeaderTypeDef CAN2_RX0_Message;
        LED_SHOW.CAN2_Recieve = 1;

        //从大疆电调获取can数据
        if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&CAN2_RX0_Message,CAN2_RxData) != HAL_OK)
        {
            Error_Handler();
        }
        
        if(CAN2_RX0_Message.IDE == CAN_ID_STD)
        {
            //更新电机数据
            get_motor_measure(&CAN2_RX0_Message, CAN2_RxData);
            
            //RM电机进行角度积分
			for(uint16_t m=0;m<8;m++)
				RM_MOTOR_Angle_Integral(&MOTO_REAL_INFO[m]);
        }

        if(CAN2_RX0_Message.IDE == CAN_ID_EXT)
        {
			test_id = 1;
            if(CONTROL.BOARD_ID==3 || CONTROL.BOARD_ID==4)
                can_get_data(&CAN2_RX0_Message,CAN2_RxData);
        }
    }
}

/**
 * @brief can命令发送函数，拓展帧
 * @param hcan 使用哪个can，hcan1 or hcan2
 * @param controller_id 使用的id
 * @param pdata 发送的数据结构体
 * @param 数据长度
 * @return NULL
*/
void comm_can_transmit_extid(CAN_HandleTypeDef* hcan, uint32_t ExtId, uint8_t *pdata, uint8_t length)
{
    CAN_TxHeaderTypeDef			TxHeader;   //can数据发送句柄
	uint32_t txmailbox;
	
    if (length > 8)
    {
        length = 8;
    }
	TxHeader.DLC = (uint8_t)8 ;     //数据长度
	TxHeader.RTR = CAN_RTR_DATA ;   
	TxHeader.IDE = CAN_ID_EXT ;     //使用拓展帧
	TxHeader.ExtId = ExtId; //VESC电调id
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)    //等待发送邮箱空
        HAL_Delay(1);                                       //防止炸邮箱
    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, pdata, &txmailbox)!= HAL_OK)
    {
        Error_Handler();
    }
    else
    {
        LED_SHOW.CAN2_Send = 1;
    }
}


/**
 * @brief can命令发送函数，标准帧
 * @param hcan 使用哪个can，hcan1 or hcan2
 * @param controller_id 使用的id
 * @param pdata 发送的数据结构体
 * @param 数据长度
 * @return NULL
*/
void comm_can_transmit_stdid(CAN_HandleTypeDef* hcan, uint16_t StdId,uint8_t *pdata, uint8_t length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t tx_mailbox = 0;
    if(length > 8)
        length = 8;
    
    TxHeader.StdId = StdId;         //标准标识符
    TxHeader.IDE = CAN_ID_STD;      //帧模式(标准帧或拓展帧)
    TxHeader.RTR = CAN_RTR_DATA;    //帧类型(数据帧或远程帧)
    TxHeader.DLC = length;          //数据长度
    TxHeader.TransmitGlobalTime = DISABLE;  //不发送标记时间

	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0);    //等待发送邮箱空
        HAL_Delay(1);                                       //防止炸邮箱
    if (HAL_CAN_AddTxMessage(hcan,&TxHeader,pdata,&tx_mailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

/***********************
 * 类型转换辅助函数
***********************/
//16位int数据填充
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

//16位unsigned int数据填充
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

//32位int数据填充
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index)
{
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

//32位unsigned int数据填充
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

/**
 * @brief 浮点转换函数,16位float数据填充
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index, bool unsign_flag) 
{
    if(unsign_flag)
        buffer_append_int16(buffer, (int16_t)(number * scale), index);
    else
        buffer_append_uint16(buffer, (uint16_t)(number * scale), index);
}

/**
 * @brief 浮点转换函数,32位float数据填充
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        buffer_append_uint32(buffer, (uint32_t)(number * scale), index);
    else
        buffer_append_int32(buffer, (int32_t)(number * scale), index);
}


/**
 * @brief 16位int数据获取
 * @param 数据缓存数组
*/
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                  ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


/**
 * @brief 16位uint数据获取
 * @param 数据缓存数组
*/
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index)
{
    uint16_t res = ((uint16_t)buffer[*index]) << 8 |
                   ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


/**
 * @brief 32位int数据获取
 * @param 数据缓存数组
*/
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index)
{
    int32_t res = ((uint32_t)buffer[*index]) << 24      |
                  ((uint32_t)buffer[*index + 1]) << 16  |
                  ((uint32_t)buffer[*index + 2]) << 8   |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


/**
 * @brief 16位uint数据获取
 * @param 数据缓存数组
*/
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index)
{
    uint32_t res = ((uint32_t)buffer[*index]) << 24     |
                   ((uint32_t)buffer[*index + 1]) << 16 |
                   ((uint32_t)buffer[*index + 2]) << 8  |
                   ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


/**
 * @brief 浮点转换函数,16位float数据获取
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        return (float)buffer_get_uint16(buffer, index) / scale;
    else
        return (float)buffer_get_int16(buffer, index) / scale;
}


/**
 * @brief 浮点转换函数,32位float数据填充
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned,使用置1，否则置0
*/
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        return (float)buffer_get_uint32(buffer, index) / scale;
    else
        return (float)buffer_get_int32(buffer, index) / scale;
}
