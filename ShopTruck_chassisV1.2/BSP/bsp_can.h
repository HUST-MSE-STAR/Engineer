/* 
 * FileName - The C head file of the bsp_can.c driver
 * NOTE: This file is based on HAL library of stm32 platform
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes           mail
 * 2020-03-20     ElonJoker         first version   2649853081@qq.com
 */
#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"
#include "mytype.h"
#include "pid.h"

#define FILTER_BUF_LEN		5
#define IDMARK_ONE_FOUR  0x200
#define IDMARK_FIVE_EIGHT 0X1FF


/*接收到的云台电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;

/*CAN发送或是接收的ID*/
typedef enum
{
	CAN_3508Moto_ALL_ID = 0x200,
	CAN_3508_L1_ID = 0x201,
	CAN_3508_L2_ID = 0x202,
	CAN_3508_R1_ID = 0x203,
	CAN_3508_R2_ID = 0x204,
	CAN_2006_under_ID = 0x205,
	CAN_3508_up_ID = 0x206,
	
}CAN_Message_ID;

extern moto_measure_t  moto_chassis[6];//底盘板控制的4个电机的信息储存位置
extern CAN_FilterTypeDef hCAN1_Filter; //CAN1滤波器

void userCanInit(void);
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan);

void setMotoSpeed(CAN_HandleTypeDef *hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4,int16_t IDRange);
void get_moto_measure(moto_measure_t *moto_chassis, uint8_t RxData[]);
	
#endif

