#ifndef  _MYTASK_H
#define  _MYTASK_H

#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "tim.h"

#define NORMAL_MODE    '1'
#define GIVEBULLET     '2'
#define ROLL_MODE      '3'
#define GETBULLET_1_MODE     '4'
#define GETBULLET_10_MODE    '5'
#define GETBULLET_4_MODE     '6'
#define RESCUE_MODE     '7'
#define OPEN_MODE       '8'
#define RESCUE_CAR_MODE '9'
#define GIVEBULLTE  '9'

int direction=0;
int set_speed,set_angel;
uint8_t actionSignal='0';
uint8_t masge[7];

/*任务函数*/
void takeBullteTaskFunction(void const * argument);
void giveBullteBullteTaskFunction(void const * argument);
void holderTaskFunction(void const * argument);

/*模块功能函数*/
int jugement(void);
void take(void);
#endif
