#ifndef __BODY_CONTROLLERS_H
#define __BODY_CONTROLLERS_H

#include "main.h"

typedef enum
{
    TAKE_BALL=20,
    SAVE_BALL,
    THROW_BALL,
	INVERTED,
    CONTROLLER_OFF
}BALL_STATUE;

typedef enum
{
	OPEN=23,
	CLOSE
}DOOR;



void baffle_control(DOOR door);
int16_t ball_filter(BALL_STATUE EXPECT_BALL_STA);

#endif
