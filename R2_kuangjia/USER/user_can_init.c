#include "user_can_init.h"

/**
 * @brief 配置CAN滤波器
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
 * @brief 配置CAN滤波器
*/
void User_Open_Can2(void)
{
	CAN_FilterTypeDef Filter;
	
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

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}
}



uint8_t RxData[8];
uint8_t RxData2[8];
uint16_t test_id;
/**
 * @brief hal库CAN回调函数,接收电机数据
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	// test_id = 2;	//纪念一下第一次使用3508电机
	// if(hcan == &hcan1)
	// {
	// 	CAN_RxHeaderTypeDef CAN1_RX0_message;

	// 	//获取电机数据，&msg中包含接收到的can的ID信息 (RxData 包含电机的状态信息)
	// 	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_RX0_message, RxData);
		
	// 	if (CAN1_RX0_message.IDE == CAN_ID_STD)
	// 	{
	// 		//更新电机数据
	// 		get_motor_measure(&CAN1_RX0_message, RxData);
			
	// 		//RM电机进行角度积分
	// 		for(uint16_t m=0;m<8;m++)
	// 			RM_MOTOR_Angle_Integral(&MOTO_REAL_INFO[m]);
	// 	}
	// }

	if(hcan == &hcan1)
	{
		CAN_RxHeaderTypeDef CAN1_RX0_message;
		
		//获取电机数据，&msg中包含接收到的can的ID信息 (RxData 包含电机的状态信息)
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&CAN1_RX0_message, RxData);

		if(CAN1_RX0_message.IDE == CAN_ID_STD)
		{
			test_id = 2;
			//更新电机数据
			get_rm_board_msg(&CAN1_RX0_message,RxData);
		}
	}

}


/**
 * @brief can命令发送函数
 * @param VESC电调的ID
 * @param 发送的数据结构体
 * @param 数据长度
 * @return NULL
*/
void comm_can_transmit_extid(CAN_HandleTypeDef* hcan, uint32_t controller_id, uint8_t *pdata, uint8_t length)
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
	TxHeader.ExtId = controller_id; //VESC电调id
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)    //等待发送邮箱空
        HAL_Delay(1);                                       //防止炸邮箱
    if (HAL_CAN_AddTxMessage(hcan, &TxHeader, pdata, &txmailbox)!= HAL_OK)
    {
        Error_Handler();
    }
}

void comm_can_transmit_stdid(CAN_HandleTypeDef* hcan, uint16_t StdId,uint8_t *pdata, uint8_t length)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t tx_mailbox = 0;
    if(length > 8)
        return;
    
    TxHeader.StdId = StdId;         //标准标识符
    TxHeader.IDE = CAN_ID_STD;      //帧模式(标准帧或拓展帧)
    TxHeader.RTR = CAN_RTR_DATA;    //帧类型(数据帧或远程帧)
    TxHeader.DLC = length;          //数据长度
    TxHeader.TransmitGlobalTime = DISABLE;  //不发送标记时间

	while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)    //等待发送邮箱空
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
 * @brief 浮点转换函数
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
 * @brief 浮点转换函数
 * @param 数据缓存数组
 * @param 数据
 * @param 数量级
 * @param unsign_flag 是否使用unsigned
*/
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        buffer_append_uint32(buffer, (uint32_t)(number * scale), index);
    else
        buffer_append_int32(buffer, (int32_t)(number * scale), index);
}


int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                  ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index)
{
    uint16_t res = ((uint16_t)buffer[*index]) << 8 |
                   ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}


int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index)
{
    int32_t res = ((uint32_t)buffer[*index]) << 24      |
                  ((uint32_t)buffer[*index + 1]) << 16  |
                  ((uint32_t)buffer[*index + 2]) << 8   |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index)
{
    uint32_t res = ((uint32_t)buffer[*index]) << 24     |
                   ((uint32_t)buffer[*index + 1]) << 16 |
                   ((uint32_t)buffer[*index + 2]) << 8  |
                   ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}


float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        return (float)buffer_get_uint16(buffer, index) / scale;
    else
        return (float)buffer_get_int16(buffer, index) / scale;
}


float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag)
{
    if(unsign_flag)
        return (float)buffer_get_uint32(buffer, index) / scale;
    else
        return (float)buffer_get_int32(buffer, index) / scale;
}
