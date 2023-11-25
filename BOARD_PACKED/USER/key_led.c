#include "key_led.h"

LED_SHW LED_SHOW;

void Board_Id_Set(void)
{
    if(KEY1==0 && KEY2==0)
    {
        CONTROL.BOARD_ID = BOARD_RM1;
    }
    else if(KEY1==0 && KEY2==1)
    {
        CONTROL.BOARD_ID = BOARD_RM2;
    }
    else if(KEY1==1 && KEY2==0)
    {
        CONTROL.BOARD_ID = BOARD_VESC1;
    }
    else
    {
        CONTROL.BOARD_ID = BOARD_VESC2;
    }
}


void can_on_off_led(void)
{
    if(LED_SHOW.CAN1_Recieve == 1)
        LED0 = 0;
    else 
        LED0 = 1;

    if(LED_SHOW.CAN1_Send == 1)
        LED1 = 0;
    else
        LED1 = 1;
    
    if(LED_SHOW.CAN2_Recieve == 1)
        LED2 = 0;
    else
        LED2 = 1;
    
    if(LED_SHOW.CAN2_Send == 1)
        LED3 = 0;
    else
        LED3 = 1;
}

