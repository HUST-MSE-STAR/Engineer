/* 
 * FileName - The C head file of the bsp_masge.c driver
 *       
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes           mail
 * 2020-08-17     ElonJoker         version1.0      2649853081@qq.com
 */
 
#ifndef  _BSP_MASGE_H
#define  _BSP_MASGE_H

#include "mytype.h"
#include "myTask.h"

#define SetBit(x, y)   (x |= (1<<y))      // �ض�λ��1,xΪĿ��,yΪ�ڼ�λ
#define ClearBit(x, y) (x &= ~(1<<y))   // �ض�λ��0
#define GetBit(x, y)   (x &= (1<<y))      // �ض�λȡֵ
#define ReveBit(x, y)  (x ^= (1<<y))     // �ض�λȡ��

uint8_t deal_masge_put(int mode,int up_finish,int take_finish,int rescue);
void deal_masge_get(uint8_t masge);

#endif
