#ifndef __KEY_LED_H
#define __KEY_LED_H

#include "main.h"


typedef struct LED_SHW
{
    uint8_t CAN1_Send;
    uint8_t CAN1_Recieve;
    uint8_t CAN2_Send;
    uint8_t CAN2_Recieve;
}LED_SHW;



#define PCout(n)	*(volatile uint32_t *)(0x42000000 + ((uint32_t)&GPIOC->ODR - 0x40000000)*32+n*4)
#define PBin(n)		*(volatile uint32_t *)(0x42000000 + ((uint32_t)&GPIOE->IDR - 0x40000000)*32+n*4)
#define KEY1 PBin(0)
#define KEY2 PBin(1)
#define LED0 PCout(0)
#define LED1 PCout(1)
#define LED2 PCout(2)
#define LED3 PCout(3)

void Board_Id_Set(void);
void can_on_off_led(void);

extern LED_SHW LED_SHOW;

#endif
