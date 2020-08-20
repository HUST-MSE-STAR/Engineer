/* 
 * MYTASK.C - The C file of the bsp_init.h
 * NOTE: 
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes             mail
 * 2020-08-15     ElonJoker         first version     2649853081@qq.com  
 */

#include "bsp_init.h"

/**
    * @brief  用于初始化所有的参数
    * @note   便于系统得更新参数和初始化
    * @author 占建
    * @param  None
    * @retval 
    */
void init_all(void)
 {
	userCanInit();//初始化Can参数
	dbus_uart_init();//初始化遥控器
/*PID初始化*/	
	for(int i=0; i<4; i++)//底盘电机的PID
		{
			PID_struct_init(&pid_spd[i],POSITION_PID,8000,8000,25,0.05,0);
		}
/*救援卡电机PID,角度PID加上速度PID*/		
	PID_struct_init(&pid_spd[4],POSITION_PID,3000,0,0.20,0,0);
	PID_struct_init(&pid_rescue[0],POSITION_PID,3000,2000,10,0.15,0);
		
	PID_struct_init(&pid_spd[5],POSITION_PID,9000,9000,25,0.05,0);//抬升电机PID
	PID_struct_init(&pid_rescue[1],POSITION_PID,8000,2000,10,0.15,0);
	set_spd[0]=set_spd[1]=set_spd[2]=set_spd[3]=set_spd[4]=set_spd[5]=0;//初始化速度设置为零，防止速度残留，这一步还有待商榷，找不到是否有效果
	
/*电机角度初始化*/
	for(int i=0;i<6;i++)//对于电机的角度控制极为重要
		get_moto_offset(&moto_chassis[i],&hcan1);
	 
 }
