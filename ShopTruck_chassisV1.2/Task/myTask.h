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
#include "RC.h"
#include "bsp_ramp.h"
#include "bsp_masge.h"

#define NORMAL_MODE    1
#define GIVEBULLET     2
#define CLIMB_MODE      3
#define GETBULLET_1_MODE     4
#define GETBULLET_10_MODE    5
#define GETBULLET_4_MODE     6
#define RESCUE_MODE     7
#define OPEN_MODE       8 //�ſ�צ��û�б�Ҫ���˳���Ԯģʽ�ͻ��צ���ſ�����Ԯ���ջ�
#define RESCUE_CAR_MODE 9

static int  set_spd[6];//�����ٶ�
static int mode=1;//������ѡ���ģʽ
static int key_sta,key_cnt;//������״̬�Ͱ��´���
static float vx = 0,vy = 0,wz = 0;  //ң�����������ٶ�
extern int up_finish;  
extern int receive_masge[4];
extern uint8_t RXmasge;
extern uint8_t TXmasge;

/*������*/
void moveTaskFunction(void const * argument);
void rescueTaskeFunction(void const * argument);
void msgSendTaskFunction(void const * argument); 
void upTaskeFunction(void const * argument);
void giveTaskeFunction(void const * argument);

/*��ͨ����*/
void pidDelay(int tim_ms,int id,int speed);
void masge_deal(int mode,int up_finish,int take_finish,int rescue);
    
#endif
