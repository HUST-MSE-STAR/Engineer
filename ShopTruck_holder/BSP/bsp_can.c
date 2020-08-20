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
 * 2020-07-26     ElonJoker         ���CAN����ѭ������Ϣ����   2649853081@qq.com
 */
#include "bsp_can.h"
//CAN_RxHeaderTypeDef hCAN_RxHeader; //CAN������Ϣ�Ľṹ��
CAN_FilterTypeDef hCAN_Filter; //CAN�˲���

moto_measure_t  moto_chassis[5];//���̰���Ƶ�����������Ϣ����λ��

int total_angel_0 = 0;

/**
    * @brief  CAN�ĳ�ʼ����
    * @note   
    * @author ռ��
    * @param  
    * @retval None
    */
void userCanInit(void){
		//��ʼ���˲���CAN
	hCAN_Filter.FilterActivation = ENABLE;// ʹ�ܹ�����
	hCAN_Filter.FilterBank = 0;  //���������
	hCAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK; //���� ģʽ
	hCAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;//ID 32λ��
	hCAN_Filter.FilterIdHigh = 0x0000;
	hCAN_Filter.FilterIdLow = 0x0000;
	hCAN_Filter.FilterMaskIdHigh = 0x0000;
	hCAN_Filter.FilterMaskIdLow = 0x0000;
	hCAN_Filter .FilterFIFOAssignment= CAN_RX_FIFO0;

	HAL_CAN_ConfigFilter(&hcan1, &hCAN_Filter);//����hcan1���˲���
	HAL_CAN_Start(&hcan1);	//����hcan1
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�֪ͨ
	
	hCAN_Filter.SlaveStartFilterBank = 14;//���ڷֹ�����������CAN��can1(0-13)��can2(14-27)�ֱ�õ�һ���filter
	hCAN_Filter.FilterBank = 0;  //���������
	HAL_CAN_ConfigFilter(&hcan2, &hCAN_Filter);//����hcan2���˲���
	HAL_CAN_Start(&hcan2);	//����hcan2
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);//ʹ���ж�֪ͨ
}

/**
    * @brief  ͨ��CAN���õ���
    * @note   
    * @author ռ��
    * @param  None
    * @retval 
    */
void setMotoSpeed(CAN_HandleTypeDef *hcan,int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4,int16_t IDRange)
{
	uint8_t Data[8];
	CAN_TxHeaderTypeDef hCAN_TxHeader; //������Ϣ�ľ��
	uint32_t send_mail;
		//��ʼ�����͵ľ��
	hCAN_TxHeader.StdId = IDRange;//���ݲ�ͬ�ı�ʶ�������������ID������1-4����5-8��
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
	//can�������ݵĺ���
	HAL_CAN_AddTxMessage(hcan,&hCAN_TxHeader,Data,&send_mail);


}


/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *moto_chassis, CAN_HandleTypeDef* hcan)
  * @Brief    ������̨���,3510���ͨ��CAN����������Ϣ
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *moto_chassis, uint8_t RxData[])
{				
	    moto_chassis->last_angle = moto_chassis->angle;
	 	// hcan canͨ�Ź�������Ϣ������Э��ȡ����Ӧ����	
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
	/*����ĳ�ʼ�����ݶ�ȡ�е����⣬Ӧ��ֱ�����þͺ���*/
//	uint8_t RxData[8];//��ʱ����CAN��Ϣ
//	CAN_RxHeaderTypeDef hCAN_RxHeader; //CAN������Ϣ�Ľṹ��
//	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hCAN_RxHeader, RxData);//������Ϣ
//	
//	ptr->angle = (uint16_t)(RxData[0]<<8 | RxData[1]) ;
	ptr->offset_angle = ptr->angle;
}

/**
*@bref ����ϵ�Ƕ�=0�� ֮���������������3510�������Կ�����Ϊ0������ԽǶȡ�
	*/
 void get_total_angle(moto_measure_t *p){
	
	int res1, res2, delta;
	if(p->angle < p->last_angle){			//���ܵ����
		res1 = p->angle + 8192 - p->last_angle;	//��ת��delta=+
		res2 = p->angle - p->last_angle;				//��ת	delta=-
	}else{	//angle > last
		res1 = p->angle - 8192 - p->last_angle ;//��ת	delta -
		res2 = p->angle - p->last_angle;				//��ת	delta +
	}
	//��������ת���϶���ת�ĽǶ�С���Ǹ������``
	if(ABS(res1)<ABS(res2))
		delta = res1;//���������ö������仯��
	else
		delta = res2;

	p->total_angle += delta;
	p->last_angle = p->angle;
}


/*******************************************************************************************
  * @Func			void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    ����һ���ص�����,������������������ѡ�����ʱ��FIFO/FIFIO1��Ӧ����������ͬ�Ļص�����
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{   
	 uint8_t RxData[8];//��ʱ����CAN��Ϣ
	 CAN_RxHeaderTypeDef hCAN_RxHeader; //CAN������Ϣ�Ľṹ��
   HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &hCAN_RxHeader, RxData);//������Ϣ
	
	//�Բ�ͬ��CAN���߸���Э�齫can���ص����ݽ�������
	  if(_hcan==&hcan1){
			switch(hCAN_RxHeader.StdId){//���ݽ��յı�ʶ��
				case CAN_3508_L1_ID:
				case CAN_3508_L2_ID:
				case CAN_3508_R1_ID:
				case CAN_3508_R2_ID:
				case CAN_2006_under_ID:
						get_moto_measure(&moto_chassis[hCAN_RxHeader.StdId-CAN_3508Moto_ALL_ID-1],RxData);//����ID��ȡ����±겢������Ϣ
						break;			
	        }			
	}
	
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IER_FMPIE0);//�����ܽ���ж�ֻ�ܽ�һ�ε�ë��
}
