/* 
 * BSP_CAN.C - The C  file of the BSP_MASGE driver
 * NOTE: 用于处理串口接收数据紊乱
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes                       mail
 * 2020-08-17     ElonJoker         first version               2649853081@qq.com
 */
 
#include "bsp_masge.h"

/**
    * @brief  将信息存入一个字符
    * @note   
    * @author 占建
    * @param  None
    * @retval 
    */
uint8_t deal_masge_put(int mode,int up_finish,int take_finish,int rescue)
{
	int postion =0;
	uint8_t masge=0;
	/*处理mode,将mode放在3-6 bit位上*/
		for(int i=0;i<4;i++)
		{  
			postion = i+3;//3-6位作为mode的位置
			if(mode%2==1)
			SetBit(masge,postion);
			else
			ClearBit(masge,postion);
			mode/=2;
		}
	/*处理up_finish,在2bit位*/  
		if(up_finish == 1)
			SetBit(masge,2);
		else
			ClearBit(masge,2);
	/*处理take_finish,在1bit位*/  	
		if(take_finish == 1)
			SetBit(masge,1);
		else
			ClearBit(masge,1);
	/*处理rescue,在0bit位*/  	
		if(rescue == 1)
			SetBit(masge,0);
		else
			ClearBit(masge,0);	
  return masge;		
}

/**
    * @brief  将信息取出
    * @note   
    * @author 占建
    * @param  None
    * @retval 
    */
void deal_masge_get(uint8_t masge)
{
	int postion =0,copy_mode=0;
	char copy;
	
		copy=masge;
		receive_masge[0]= GetBit(copy,0);
	
		copy=masge;
		receive_masge[1]= GetBit(copy,1)/2;
	
		copy=masge;
		receive_masge[2]= GetBit(copy,2)/4;	
	/*得到mode*/
	for(int i=0;i<4;i++)
	{
		copy=masge;
		postion=i+3;
		copy_mode+= GetBit(copy,postion)/8;
	}
	receive_masge[3]= copy_mode;
}
