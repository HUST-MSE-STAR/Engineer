#ifndef  _MYTASK_H
#define  _MYTASK_H

#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "RC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tim.h"

#define NORMAL_MODE    '1'
#define GIVEBULLET     '2'
#define CLIMB_MODE      '3'
#define GETBULLET_1_MODE     '4'
#define GETBULLET_10_MODE    '5'
#define GETBULLET_4_MODE     '6'
#define RESCUE_MODE     '7'
#define OPEN_MODE       '8'
#define RESCUE_CAR_MODE '9'


static int key_sta,key_cnt;
static int  set_spd[9];//设置速度
static float vx,vy,wz;  //速度
static uint8_t test='1';
static uint8_t mode='1';//传输所选择的模式
static uint8_t actionSignal='0';
static int move_step=2;   //取弹时，底盘的移动倍率

void getBulletTaskFunction(void const * argument);
void msgSendTaskFunction(void const * argument); 
void moveTaskFunction(void const * argument);
void rescueTaskeFunction(void const * argument);
//void climbUPIslandFunction(void const * argument);
void giveBulletTaskFunction(void const * argument);
//void climbDownIslandFunction(void const * argument);

#endif
