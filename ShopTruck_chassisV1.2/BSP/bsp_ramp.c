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
															.target_val = 0,			//目标参考值
															.present_ref = 0,			//当前参考值
															.step_val = 0,				//当前参考值到目标参考值所需要的步数
															.inc_val = 0,				//步长
                              };
 
 /**
     * @brief  斜波函数输出
     * @note   用于计算斜波函数的输出值
     * @author 占建
     * @param  speed_ramp_mod_t p  斜波函数结构体
     * @retval int16_t ref 输出设定参考值
     */ 
int32_t speed_ramp_calc(speed_ramp_mod_t *p)
 {
	int32_t ref;
	ref = p->present_ref;
/*根据步长来确定每次增加的数值*/	
	if(p->step_val > 1)
		{
		ref += p->inc_val;//按照既定的步长进行增加的操作
		p->step_val--;//步数减少
	  }
	else 
		if(p->step_val == 1)//最后一步直接到达设定值
		{
			ref = p->target_val;
			p->step_val = 0;
	   }

	p->present_ref = ref;
	return ref;
 }

 
 /**
    * @brief  斜波函数初始化
    * @note   
    * @author 占建 
    * @param  speed_ramp_mod_t *p  斜波函数结构体
              int16_t target_val   目标数值
              uint16_t step_val    步数
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
		//计算步长度
		p->step_val = (int32_t)step_val;
		p->inc_val = (p->target_val - ref)/p->step_val;
	}
		
	}

/**
      * @brief  判断是否完成了斜波函数的计算
      * @note   
      * @author 占建
      * @param  speed_ramp_mod_t *p   斜波函数结构体
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
