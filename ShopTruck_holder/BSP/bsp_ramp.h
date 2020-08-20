/* 
 * FileName - The C head file of the bsp_ramp.c driver
 * NOTE: This file is based on HAL library of stm32 platform
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes           mail
 * 2020-08-14     ElonJoker         first version   2649853081@qq.com
 */

#ifndef __BSP_RAMP
#define __BSP_RAMP

#include "mytype.h"

#define STEP_VAL 10

typedef struct speed_ramp_mod{
	int32_t target_val;		//Ŀ��ο�ֵ
	int32_t present_ref;	//��ǰ�ο�ֵ
	int32_t step_val;		//��ǰ�ο�ֵ��Ŀ��ο�ֵ����Ҫ�Ĳ���
	int32_t inc_val;		//����/б��
 }speed_ramp_mod_t;

extern speed_ramp_mod_t user_ramp;
	
int32_t speed_ramp_calc(speed_ramp_mod_t *p);
void speed_ramp_init(speed_ramp_mod_t *p,int32_t target_val,uint32_t step_val);
 uint8_t speed_ramp_completed(speed_ramp_mod_t *p);
															
#endif					
