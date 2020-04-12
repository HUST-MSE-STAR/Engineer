#include "bsp_can.h"


CAN_RxHeaderTypeDef hCAN1_RxHeader; //CAN1接收消息
CAN_FilterTypeDef hCAN1_Filter; //CAN1滤波器
moto_measure_t  moto_chassis[4];//底盘四个电机的信息储存位置
moto_measure_t  moto_info;//底盘四个电机的信息储存位置
uint8_t RxData[8];

void userCanInit(CAN_HandleTypeDef *hcan)
{ 

	
	//初始化滤波器
	hCAN1_Filter.FilterBank = 0;  //等同于官方的FilterNumber，用于配置过滤器组号
	hCAN1_Filter.FilterMode = CAN_FILTERMODE_IDMASK; //屏蔽 模式
	hCAN1_Filter.FilterScale = CAN_FILTERSCALE_32BIT;//ID 32位的
	hCAN1_Filter.FilterIdHigh = 0x0000;
	hCAN1_Filter.FilterIdLow = 0x0000;
	hCAN1_Filter.FilterMaskIdHigh = 0x0000;
	hCAN1_Filter.FilterMaskIdLow = 0x0000;
	hCAN1_Filter .FilterFIFOAssignment= CAN_FilterFIFO0;
//	hCAN1_Filter.SlaveStartFilterBank = 14;//等同于官方BankNumber,用于分过滤器给两个CAN，can1(0-13)和can2(14-27)分别得到一半的filter
	hCAN1_Filter.FilterActivation = ENABLE;
	
	//配置hcan的滤波器
	HAL_CAN_ConfigFilter(hcan, &hCAN1_Filter);
	
	//启动hcan
	HAL_CAN_Start(hcan);
			
	//使能中断通知
	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}


void setMotoSpeed(CAN_HandleTypeDef *hcan,uint16_t iq1, uint16_t iq2, uint16_t iq3, uint16_t iq4)
{
	uint8_t Data[8]={0};
	CAN_TxHeaderTypeDef hCAN1_TxHeader; //CAN1发送消息
		//初始化发送的句柄
	hCAN1_TxHeader.StdId = 0x200;
	hCAN1_TxHeader.IDE = CAN_ID_STD;
	hCAN1_TxHeader.RTR = CAN_RTR_DATA;
	hCAN1_TxHeader.DLC = 0x08;
	//hCAN1_TxHeader.TransmitGlobalTime    =    ENABLE;
	
	Data[0] = (iq1 >> 8)&0xff;
	Data[1] = iq1&0xff;
	Data[2] = (iq2 >> 8)&0xff;
	Data[3] = iq2&0xff;
	Data[4] = (iq3 >> 8)&0xff;
	Data[5] = iq3&0xff;
  Data[6] = (iq4 >> 8)&0xff;
	Data[7] = iq4&0xff;
	//HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox)
	//can发送数据的函数

	HAL_CAN_AddTxMessage(hcan,&hCAN1_TxHeader,Data,(uint32_t*)CAN_TX_MAILBOX0);

}


/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *moto_chassis, CAN_HandleTypeDef* hcan)
  * @Brief    接收云台电机,3510电机通过CAN发过来的信息
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *moto_chassis, CAN_HandleTypeDef* hcan)
{

	if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hCAN1_RxHeader, RxData)==HAL_ERROR)// CAN_RX_FIFO0 第二个参数貌似是进中断的一个判断
	{					
	moto_chassis->last_angle = moto_chassis->angle;

	// hcan can通信过来的信息，根据协议取出对应数据
	
	moto_chassis->angle = (uint16_t)(RxData[0]<<8 | RxData[1]) ;
	moto_chassis->real_current  = (int16_t)(RxData[2]<<8 | RxData[3]);
	moto_chassis->speed_rpm = moto_chassis->real_current;	
	moto_chassis->given_current = (int16_t)(RxData[4]<<8 | RxData[5])/-5;
	moto_chassis->hall = RxData[6];
		
	if(moto_chassis->angle - moto_chassis->last_angle > 4096)
		moto_chassis->round_cnt --;
	else if (moto_chassis->angle - moto_chassis->last_angle < -4096)
		moto_chassis->round_cnt ++;
	moto_chassis->total_angle = moto_chassis->round_cnt * 8192 + moto_chassis->angle - moto_chassis->offset_angle;
}
	
}
/*this function should be called after system+can init */
void get_moto_offset(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
{
	ptr->angle = (uint16_t)(RxData[0]<<8 | RxData[1]) ;
	ptr->offset_angle = ptr->angle;
}



#define ABS(x)	( (x>0) ? (x) : (-x) )
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
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    这是一个回调函数,都不用声明
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
{   
	int i=0;
	//ignore can1 or can2.根据协议将can传回的数据解析处理
//	switch(hCAN1_RxHeader.StdId){//
//		case CAN_3510Moto1_ID:
//		case CAN_3510Moto2_ID:
//		case CAN_3510Moto3_ID:
//		case CAN_3510Moto4_ID:
//			{
//				static uint8_t i;
//				i = hCAN1_RxHeader.StdId - CAN_3510Moto1_ID;//相减得到电机号1/2/3/4
//				//根据msg_cnt的数量判断执行
//				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], _hcan) : get_moto_measure(&moto_chassis[i], _hcan);
//				get_moto_measure(&moto_info, _hcan);

//				//get_moto_measure(&moto_chassis[i], _hcan);
//			}
//			break;		
//	}			 
	  for(i=0;i<4;i++)//得到电机
    get_moto_measure(&moto_chassis[i], _hcan);
    __HAL_CAN_ENABLE_IT(&hcan1, CAN_IER_FMPIE0);//解决中断只能进一次的毛病
    
}

