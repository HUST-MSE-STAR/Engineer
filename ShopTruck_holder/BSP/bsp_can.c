/* 
 * BSP_CAN.C - The C  file of the BSP_CAN driver
 * NOTE: This file is based on HAL library of stm32 platform
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes                       mail
 * 2020-07-10     ElonJoker         first version               2649853081@qq.com
 * 2020-07-26     ElonJoker         解决CAN不能循环发消息问题   2649853081@qq.com
 */
#include "bsp_can.h"
//CAN_RxHeaderTypeDef hCAN_RxHeader; //CAN接收消息的结构体
CAN_FilterTypeDef hCAN_Filter; //CAN滤波器

moto_measure_t  moto_chassis[5];//底盘板控制的五个电机的信息储存位置

int total_angel_0 = 0;

/**
    * @brief  CAN的初始配置
    * @note   
    * @author 占建
    * @param  
    * @retval None
    */
void userCanInit(void){
		//初始化滤波器CAN
	hCAN_Filter.FilterActivation = ENABLE;// 使能过滤器
	hCAN_Filter.FilterBank = 0;  //过滤器编号
	hCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK; //屏蔽 模式
	hCAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;//ID 32位的
	hCAN_Filter.FilterIdHigh = 0x0000;
	hCAN_Filter.FilterIdLow = 0x0000;
	hCAN_Filter.FilterMaskIdHigh = 0x0000;
	hCAN_Filter.FilterMaskIdLow = 0x0000;
	hCAN_Filter .FilterFIFOAssignment= CAN_RX_FIFO0;

	HAL_CAN_ConfigFilter(&hcan1, &hCAN_Filter);//配置hcan1的滤波器
	HAL_CAN_Start(&hcan1);	//启动hcan1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断通知
	
	hCAN_Filter.SlaveStartFilterBank = 14;//用于分过滤器给两个CAN，can1(0-13)和can2(14-27)分别得到一半的filter
	hCAN_Filter.FilterBank = 0;  //过滤器编号
	HAL_CAN_ConfigFilter(&hcan2, &hCAN_Filter);//配置hcan2的滤波器
	HAL_CAN_Start(&hcan2);	//启动hcan2
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//使能中断通知
}

/**
    * @brief  通过CAN设置电流
    * @note   
    * @author 占建
    * @param  None
    * @retval 
    */
void setMotoSpeed(CAN_HandleTypeDef *hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4,int16_t IDRange)
{
	uint8_t Data[8];
	CAN_TxHeaderTypeDef hCAN_TxHeader; //发送消息的句柄
	uint32_t send_mail;
		//初始化发送的句柄
	hCAN_TxHeader.StdId = IDRange;//根据不同的标识符来决定电机的ID控制是1-4还是5-8号
	hCAN_TxHeader.IDE = CAN_ID_STD;
	hCAN_TxHeader.RTR = CAN_RTR_DATA;
	hCAN_TxHeader.DLC = 0x08;
	
	Data[0] = (uint8_t)((iq1 >> 8)&0xff);
	Data[1] = (uint8_t)(iq1&0xff);
	Data[2] = (uint8_t)((iq2 >> 8)&0xff);
	Data[3] = (uint8_t)(iq2&0xff);
	Data[4] = (uint8_t)((iq3 >> 8)&0xff);
	Data[5] = (uint8_t)(iq3&0xff);
  Data[6] = (uint8_t)((iq4 >> 8)&0xff);
	Data[7] = (uint8_t)(iq4&0xff);
	//HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
	//can发送数据的函数
	HAL_CAN_AddTxMessage(hcan,&hCAN_TxHeader,Data,&send_mail);


}


/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *moto_chassis, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *moto_chassis, uint8_t RxData[])
{				
	    moto_chassis->last_angle = moto_chassis->angle;
	 	// hcan can通信过来的信息，根据协议取出对应数据	
			moto_chassis->angle = (uint16_t)(RxData[0]<<8 | RxData[1]) ;
			moto_chassis->real_current  = (int16_t)(RxData[2]<<8 | RxData[3]);
			moto_chassis->speed_rpm = moto_chassis->real_current;	
			moto_chassis->given_current = (int16_t)(RxData[4]<<8 | RxData[5]);
			moto_chassis->hall = RxData[6];
				
			if(moto_chassis->angle - moto_chassis->last_angle > 4096)
				moto_chassis->round_cnt --;
			else if (moto_chassis->angle - moto_chassis->last_angle < -4096)
				moto_chassis->round_cnt ++;
			moto_chassis->total_angle = moto_chassis->round_cnt * 8192 + moto_chassis->angle - moto_chassis->offset_angle;
	
}
///*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	/*这里的初始化数据读取有点问题，应该直接设置就好了*/
//	uint8_t RxData[8];//临时接收CAN信息
//	CAN_RxHeaderTypeDef hCAN_RxHeader; //CAN接收消息的结构体
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hCAN_RxHeader, RxData);//接收信息
//	
//	ptr->angle = (uint16_t)(RxData[0]<<8 | RxData[1]) ;
	ptr->offset_angle = ptr->angle;
}

/**
*@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
	*/
 void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//可能的情况
		res1 = p->angle + 8192 - p->last_angle;	//正转，delta=+
		res2 = p->angle - p->last_angle;				//反转	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//反转	delta -
		res2 = p->angle - p->last_angle;				//正转	delta +
	}
	//不管正反转，肯定是转的角度小的那个是真的``
	if(ABS(res1)<ABS(res2))
		delta = res1;//看读音，得儿塔，变化量
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}


/*******************************************************************************************
  * @Func			void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    这是一个回调函数,都不用声明，但是在选择参数时候，FIFO/FIFIO1对应的是两个不同的回调函数
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{   
	 uint8_t RxData[8];//临时接收CAN信息
	 CAN_RxHeaderTypeDef hCAN_RxHeader; //CAN接收消息的结构体
   HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &hCAN_RxHeader, RxData);//接收信息
	
	//对不同的CAN总线根据协议将can传回的数据解析处理
	  if(_hcan==&hcan1){
			switch(hCAN_RxHeader.StdId){//根据接收的标识符
				case CAN_3508_L1_ID:
				case CAN_3508_L2_ID:
				case CAN_3508_R1_ID:
				case CAN_3508_R2_ID:
				case CAN_2006_under_ID:
						get_moto_measure(&moto_chassis[hCAN_RxHeader.StdId-CAN_3508Moto_ALL_ID-1],RxData);//根据ID获取电机下标并存入信息
						break;			
	        }			
	}
	
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IER_FMPIE0);//并不能解决中断只能进一次的毛病
}
