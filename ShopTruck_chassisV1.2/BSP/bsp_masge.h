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

#define SetBit(x, y)   (x |= (1<<y))      // 特定位置1,x为目标,y为第几位
#define ClearBit(x, y) (x &= ~(1<<y))   // 特定位清0
#define GetBit(x, y)   (x &= (1<<y))      // 特定位取值
#define ReveBit(x, y)  (x ^= (1<<y))     // 特定位取反

uint8_t deal_masge_put(int mode,int up_finish,int take_finish,int rescue);
void deal_masge_get(uint8_t masge);

#endif
