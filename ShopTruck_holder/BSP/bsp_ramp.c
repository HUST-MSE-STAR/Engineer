/* 
 * FileName - The C file of the bsp_ramp.c
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
 
 
#include "bsp_ramp.h"
 
 speed_ramp_mod_t user_ramp = {
															.target_val = 0,			//Ŀ��ο�ֵ
															.present_ref = 0,			//��ǰ�ο�ֵ
															.step_val = 0,				//��ǰ�ο�ֵ��Ŀ��ο�ֵ����Ҫ�Ĳ���
															.inc_val = 0,				//����
                              };
 
 /**
     * @brief  б���������
     * @note   ���ڼ���б�����������ֵ
     * @author ռ��
     * @param  speed_ramp_mod_t p  б�������ṹ��
     * @retval int16_t ref ����趨�ο�ֵ
     */ 
int32_t speed_ramp_calc(speed_ramp_mod_t *p)
 {
	int32_t ref;
	ref = p->present_ref;
/*���ݲ�����ȷ��ÿ�����ӵ���ֵ*/	
	if(p->step_val > 1)
		{
		ref += p->inc_val;//���ռȶ��Ĳ����������ӵĲ���
		p->step_val--;//��������
	  }
	else 
		if(p->step_val == 1)//���һ��ֱ�ӵ����趨ֵ
		{
			ref = p->target_val;
			p->step_val = 0;
	   }

	p->present_ref = ref;
	return ref;
 }

 
 /**
    * @brief  б��������ʼ��
    * @note   
    * @author ռ�� 
    * @param  speed_ramp_mod_t *p  б�������ṹ��
              int16_t target_val   Ŀ����ֵ
              uint16_t step_val    ����
    * @retval 
    */
void speed_ramp_init(speed_ramp_mod_t *p,int32_t target_val,uint32_t step_val)
 {
	int32_t ref = 0;
	ref = p->present_ref;
	if(step_val == 0){
		p->step_val = 0;
		p->inc_val = 0;
		p->present_ref = target_val;
	}else{
		p->target_val = target_val;
		//���㲽����
		p->step_val = (int32_t)step_val;
		p->inc_val = (p->target_val - ref)/p->step_val;
	}
		
	}

/**
      * @brief  �ж��Ƿ������б�������ļ���
      * @note   
      * @author ռ��
      * @param  speed_ramp_mod_t *p   б�������ṹ��
      * @retval 
      */	
uint8_t speed_ramp_completed(speed_ramp_mod_t *p)
	{
	uint8_t retval = 0;
	if(p->step_val == 0){
		retval = 1;
	}
	return retval;
  }	
