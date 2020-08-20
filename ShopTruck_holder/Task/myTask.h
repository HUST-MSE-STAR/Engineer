/* 
 * FileName - The C head file of the myTask.c driver
 * NOTE: This file is based on HAL library of stm32 platform
 *       
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes           mail
 * 2020-07-26     ElonJoker         version1.1      2649853081@qq.com
 */
 
 #ifndef  _MYTASK_H
#define  _MYTASK_H

#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "bsp_ramp.h"
#include "bsp_masge.h"

#define NORMAL_MODE    1
#define GIVEBULLET     2
#define CLIMB_MODE      3
#define GETBULLET_1_MODE     4
#define GETBULLET_10_MODE    5
#define GETBULLET_4_MODE     6
#define RESCUE_MODE     7
#define OPEN_MODE       8
#define RESCUE_CAR_MODE 9

#define C_1_1 17 //机械齿数
#define C_1_2 46
#define C_2_1 17
#define C_2_2 46
#define C_3_1 11
#define C_3_2 46


extern int set_speed[2],set_angel;
extern uint8_t send_masge[9];//发送给底盘得信号，0位为取弹任务是否完成,1位为补弹是否完成
static int direction=3;//取弹定位移动方向
extern uint8_t RXmasge;
extern uint8_t TXmasge;
extern int receive_masge[4];//3放置mode,2放置up,1放置take,0放置rescue

/*任务函数*/
void takeBullteTaskFunction(void const * argument);
void holderTaskFunction(void const * argument);
void giveBullteTaskFunction(void const * argument);
void rescueTaskeFunction(void const * argument);

/*模块功能函数*/
int jugement(void);
void take(int angle);

#endif
