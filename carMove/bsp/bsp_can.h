#ifndef __BSP_CAN
#define __BSP_CAN

#include "can.h"
#include "mytype.h"
#include "pid.h"

#define FILTER_BUF_LEN		5
#define IDMARK_ONE_FOUR  0x200
#define IDMARK_FIVE_EIGHT 0X1FF

/*接收到的云台电机的参数结构体*/
typedef struct{
	int16_t	 	speed_rpm;
    int16_t  	real_current;
    int16_t  	given_current;
    uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;

extern moto_measure_t  moto_chassis[9];//底盘板控制的九个电机的信息储存位置

extern CAN_TxHeaderTypeDef hCAN1_TxHeader; //CAN1发送消息
extern CAN_RxHeaderTypeDef hCAN_RxHeader; //CAN1接收消息
extern CAN_FilterTypeDef hCAN1_Filter; //CAN1滤波器
extern uint8_t RxData[8];


void userCanInit(CAN_HandleTypeDef *hcan);
void can1Init(CAN_HandleTypeDef *hcan);
void can2Init(CAN_HandleTypeDef *hcan);

void setMotoSpeed(CAN_HandleTypeDef *hcan,uint16_t iq1, uint16_t iq2, uint16_t iq3, uint16_t iq4,uint16_t IDRange);
void get_moto_measure(moto_measure_t *moto_chassis, CAN_HandleTypeDef* hcan);
	
#endif

