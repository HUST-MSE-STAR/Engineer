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
    * @brief  ���ڳ�ʼ�����еĲ���
    * @note   ����ϵͳ�ø��²����ͳ�ʼ��
    * @author ռ��
    * @param  None
    * @retval 
    */
void init_all(void)
 {
	userCanInit();//��ʼ��Can����
/*PID��ʼ��*/	
	/*��λ��һ��PIDPID��ʼ��*/  
	PID_struct_init(&pid_spd[0],POSITION_PID,3000,0,5,0,0);
	PID_struct_init(&pid_spd[4],POSITION_PID,3000,1000,10,0.15,0); 
	/*ȡ��ҩ��һ��PIDPID��ʼ��*/ 
	PID_struct_init(&pid_spd[1],POSITION_PID,3000,0,0.05,0,0);//����PID̫���ˣ��Ƕȵ�PIDֻҪһ���ͺ��ˣ�
	PID_struct_init(&pid_spd[3],POSITION_PID,3000,1000,10,0.15,0);
	
	PID_struct_init(&pid_takeBullte[0],POSITION_PID,6000,0,10,0,0);//Ϊ�������ʱ�����Ȳ��������⣬��������PID���н��
	PID_struct_init(&pid_takeBullte[1],POSITION_PID,6000,1000,10,0.15,0);
/*����Ƕȳ�ʼ��*/
	for(int i=0;i<4;i++)
		get_moto_offset(&moto_chassis[i],&hcan1);
	 
 }
