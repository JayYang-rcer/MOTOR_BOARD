#ifndef __CONFIG_H
#define __CONFIG_H

#include "main.h"

void Pid_Init_All(void);
void RM_Motor_Init(void);
void RM_SPPED_TEST(void);
void debug_safe_printf(const char *format, ...);

#endif
