/* 
 * BSP_CAN.C - The C  file of the BSP_MASGE driver
 * NOTE: ���ڴ����ڽ�����������
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
    * @brief  ����Ϣ����һ���ַ�
    * @note   
    * @author ռ��
    * @param  None
    * @retval 
    */
uint8_t deal_masge_put(int mode,int up_finish,int take_finish,int rescue)
{
	int postion =0;
	uint8_t masge=0;
	/*����mode,��mode����3-6 bitλ��*/
		for(int i=0;i<4;i++)
		{  
			postion = i+3;//3-6λ��Ϊmode��λ��
			if(mode%2==1)
			SetBit(masge,postion);
			else
			ClearBit(masge,postion);
			mode/=2;
		}
	/*����up_finish,��2bitλ*/  
		if(up_finish == 1)
			SetBit(masge,2);
		else
			ClearBit(masge,2);
	/*����take_finish,��1bitλ*/  	
		if(take_finish == 1)
			SetBit(masge,1);
		else
			ClearBit(masge,1);
	/*����rescue,��0bitλ*/  	
		if(rescue == 1)
			SetBit(masge,0);
		else
			ClearBit(masge,0);	
  return masge;		
}

/**
    * @brief  ����Ϣȡ��
    * @note   
    * @author ռ��
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
	/*�õ�mode*/
	for(int i=0;i<4;i++)
	{
		copy=masge;
		postion=i+3;
		copy_mode+= GetBit(copy,postion)/8;
	}
	receive_masge[3]= copy_mode;
}
