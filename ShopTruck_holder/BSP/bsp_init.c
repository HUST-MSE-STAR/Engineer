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
/*PID初始化*/	
	/*定位的一对PIDPID初始化*/  
	PID_struct_init(&pid_spd[0],POSITION_PID,3000,0,5,0,0);
	PID_struct_init(&pid_spd[4],POSITION_PID,3000,1000,10,0.15,0); 
	/*取弹药的一对PIDPID初始化*/ 
	PID_struct_init(&pid_spd[1],POSITION_PID,3000,0,0.05,0,0);//这里PID太大了，角度的PID只要一点点就好了，
	PID_struct_init(&pid_spd[3],POSITION_PID,3000,1000,10,0.15,0);
	
	PID_struct_init(&pid_takeBullte[0],POSITION_PID,6000,0,10,0,0);//为解决会拉时候力度不够的问题，采用两套PID进行解决
	PID_struct_init(&pid_takeBullte[1],POSITION_PID,6000,1000,10,0.15,0);
/*电机角度初始化*/
	for(int i=0;i<4;i++)
		get_moto_offset(&moto_chassis[i],&hcan1);
	 
 }
