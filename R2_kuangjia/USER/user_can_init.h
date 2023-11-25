#ifndef __USER_CAN_INIT_H
#define __USER_CAN_INIT_H
#include "main.h"

#define CAN_RxExtId 0x0000
#define CAN_TxExtId 0x0000

void comm_can_transmit_extid(CAN_HandleTypeDef* hcan, uint32_t controller_id, uint8_t *pdata, uint8_t length);	   //拓展帧发送函数
void comm_can_transmit_stdid(CAN_HandleTypeDef* hcan, uint16_t StdId,uint8_t *pdata, uint8_t length);		   //标准帧发送函数
void User_Open_Can1(void);
void User_Open_Can2(void);


//buffer函数
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag);
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index, bool unsign_flag);

void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index, bool unsign_flag);
void buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index, bool unsign_flag);

#endif
