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
	dbus_uart_init();//��ʼ��ң����
/*PID��ʼ��*/	
	for(int i=0; i<4; i++)//���̵����PID
		{
			PID_struct_init(&pid_spd[i],POSITION_PID,8000,8000,25,0.05,0);
		}
/*��Ԯ�����PID,�Ƕ�PID�����ٶ�PID*/		
	PID_struct_init(&pid_spd[4],POSITION_PID,3000,0,0.20,0,0);
	PID_struct_init(&pid_rescue[0],POSITION_PID,3000,2000,10,0.15,0);
		
	PID_struct_init(&pid_spd[5],POSITION_PID,9000,9000,25,0.05,0);//̧�����PID
	PID_struct_init(&pid_rescue[1],POSITION_PID,8000,2000,10,0.15,0);
	set_spd[0]=set_spd[1]=set_spd[2]=set_spd[3]=set_spd[4]=set_spd[5]=0;//��ʼ���ٶ�����Ϊ�㣬��ֹ�ٶȲ�������һ�����д���ȶ���Ҳ����Ƿ���Ч��
	
/*����Ƕȳ�ʼ��*/
	for(int i=0;i<6;i++)//���ڵ���ĽǶȿ��Ƽ�Ϊ��Ҫ
		get_moto_offset(&moto_chassis[i],&hcan1);
	 
 }
