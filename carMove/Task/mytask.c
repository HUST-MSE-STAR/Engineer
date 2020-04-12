#include "mytask.h"

float x;
uint8_t y='0';

void moveTaskFunction(void const * argument)
{	
  
	/* Infinite loop */	
		  for(;;)
		{
	 userCanInit(&hcan1);
   HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);//此处有待解决！！！！！！
// 对四个轮子的PID 进行调节
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
									pid_spd[1].pos_out,
									pid_spd[2].pos_out,
									pid_spd[3].pos_out,IDMARK_ONE_FOUR);			

			//按键设置电机的基础马力(速度)配置
	switch(key_sta)
	{
		case 0:	//no key
			if( 0 == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) )
			{
				key_sta = 1;
			}
			break;
		case 1: //key down wait release.
			if( 0 == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) )
			{
				key_sta = 2;
				key_cnt++;
			}
			else
			{
				key_sta = 0;
			}
			break;
		case 2: 
			if( 0 != HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) )
			{
				key_sta = 0;
			}
			break;
	}
	if(key_cnt>10)
		key_cnt = 0;
  	    	vx = (float)rc.ch2 / 660 * 800;
        vy = -(float)rc.ch1 / 660 *800;
   if(mode=='4'||mode=='5')	
		 osDelay(10);
	 else
	 {
		set_spd[0] = -(-vy + vx+ rc.wheel*5.2)*(key_cnt+1);    
		set_spd[1] = -(-vy - vx+ rc.wheel*5.2)*(key_cnt+1);
	  set_spd[2] = -( vy + vx + rc.wheel*5.2)*(key_cnt+1);
	  set_spd[3] = -( vy- vx+ rc.wheel*5.2)*(key_cnt+1);	
	 }
	//  HAL_UART_Transmit(&huart6,&test,1,50);
//设置速度			
		osDelay(10);
    }	
}

void msgSendTaskFunction(void const * argument) 
{
	for(;;)
	{
		//判断此时的遥控器的模式
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_UP))//普通模式 右1 左1
			mode=NORMAL_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_MID))//普通模式下，补弹 右1 左2
	    mode=GIVEBULLET;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_DOWN))//普通模式下，登岛 右1 左3
			mode=CLIMB_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下，一排 右2  左1
			mode=GETBULLET_1_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_MID)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下，十字 右2  左2
		   mode=GETBULLET_10_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_DOWN)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下,四角 右2  左3
		   mode=GETBULLET_4_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_DOWN))//进入救援模式 右3 左1
			mode=RESCUE_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_MID))//退出救援模式，拨弹 右3 左2
		   mode=OPEN_MODE;
    
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_DOWN))//救援卡模式，拨弹 右3 左3
		   mode=RESCUE_CAR_MODE;		
		
		//发送此时的模式和云台数据
		masge[0]='a';
		masge[6]='b';
		masge[5]=mode;
		HAL_UART_Transmit(&huart6,masge,sizeof(masge),50);
		osDelay(10);
	}
	
}

void getBulletTaskFunction(void const * argument)
{
	for(;;)
	{
		if(mode=='5'||mode=='6'||mode=='4')
		{
				set_spd[0]=set_spd[1]=set_spd[2]=set_spd[3]=0;
			//九号电机设置为抬起取弹的机构，此处设置角度最安全
			 pid_calc(&pid_spd[8],moto_chassis[8].speed_rpm,1000);
			
			HAL_UART_Receive(&huart6,&actionSignal,1,50);
			while(actionSignal!='o'){//收到取弹完成标识后放下取弹机构
				HAL_UART_Receive(&huart6,&actionSignal,1,50);
				osDelay(500);
	    }
			pid_calc(&pid_spd[8],moto_chassis[8].speed_rpm,-1000);
			
			mode=NORMAL_MODE;
			actionSignal='f';//将任务完成信号置假
	  }
}
}

/*
救援卡方案要改动
*/
void rescueTaskeFunction(void const * argument)
{
		for(;;)
	{
		 if(mode=='7'||mode=='8'||mode=='9')
		  {
			   if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1)==GPIO_PIN_RESET||HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0)==GPIO_PIN_RESET)//触碰开关
				 { 
			 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,800);//自动重载值为1000
				
		     if(mode=='9')
				 {
			      osDelay(1000);//等待1秒，确保抓住车辆
						 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
			      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,800);
					}
	    }		
			}
    else
		{
			 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,200);
			 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,200);	//收回爪子和救援卡
		}
	}
	
}

//void climbUPIslandFunction(void const * argument){
//  for(;;){  
//	if(mode==CLIMB_MODE){
//    //此处已改为角度为好		
//    pid_calc(&pid_spd[4],moto_chassis[4].angle,180*3591/187);
//		pid_calc(&pid_spd[5],moto_chassis[5].angle,180*3591/187);
//		setMotoSpeed(&hcan1,pid_spd[4].pos_out,
//		                    pid_spd[5].pos_out,
//		                    0,
//		                    0,IDMARK_FIVE_EIGHT);//抬起车身
//		//前置脚检测到台阶
//		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)!=GPIO_PIN_SET|HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)!=GPIO_PIN_SET){
//		pid_calc(&pid_spd[5],moto_chassis[5].speed_rpm,500);
//		pid_calc(&pid_spd[6],moto_chassis[6].speed_rpm,500);
//		setMotoSpeed(&hcan1,pid_spd[5].pos_out,
//		                    pid_spd[6].pos_out,
//		                    0,
//		                    0,IDMARK_FIVE_EIGHT);//前移车子
//		}
//		
//	  setMotoSpeed(&hcan1,0,0,0,0,IDMARK_FIVE_EIGHT);//前移停止，使用前轮带着跑
//		pid_calc(&pid_spd[4],moto_chassis[4].speed_rpm,-80*3591/187);
//		pid_calc(&pid_spd[4],moto_chassis[5].speed_rpm,80*3591/187);
//		setMotoSpeed(&hcan1,pid_spd[4].pos_out,
//		                    pid_spd[5].pos_out,
//		                    0,
//		                    0,IDMARK_FIVE_EIGHT);//收前脚，保持后脚
//		
//		while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)!=GPIO_PIN_SET|HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_4)!=GPIO_PIN_SET){
//				for(int i=0; i<2; i++)
//			{
//				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, 500);
//			}
//			setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
//									pid_spd[1].pos_out,
//									0,
//									0,IDMARK_ONE_FOUR);//前轮往前走
//			osDelay(100);
//		}
//			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);//放下辅助轮
//		
//		pid_calc(&pid_spd[4],moto_chassis[4].speed_rpm,-80*3591/187);
//		pid_calc(&pid_spd[4],moto_chassis[5].speed_rpm,-80*3591/187);
//		setMotoSpeed(&hcan1,pid_spd[4].pos_out,
//		                    pid_spd[5].pos_out,
//		                    0,
//		                    0,IDMARK_FIVE_EIGHT);//收起后脚
//      while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)!=GPIO_PIN_SET|HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)!=GPIO_PIN_SET){
//			   osDelay(100);
//			}
//	 }
//	  HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);//登岛结束，收起辅助轮
//  	mode=NORMAL_MODE;//退出登岛模式，进入一般模式
// }
//}

//void climbDownIslandFunction(void const * argument){
// for(;;){
//	 HAL_GPIO_WritePin(GPIOE,GPIO_PIN_12,GPIO_PIN_SET);//放下辅助轮
//	 while(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)!=GPIO_PIN_SET|HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)!=GPIO_PIN_SET){
//	  pid_calc(&pid_spd[5],moto_chassis[5].speed_rpm,-500);
//		pid_calc(&pid_spd[6],moto_chassis[6].speed_rpm,-500);
//		setMotoSpeed(&hcan1,pid_spd[5].pos_out,
//		                    pid_spd[6].pos_out,
//		                    0,
//		                    0,IDMARK_FIVE_EIGHT);//后移车子
//	 }

//	 	pid_calc(&pid_spd[4],moto_chassis[4].speed_rpm,-80*3591/187);
//		pid_calc(&pid_spd[4],moto_chassis[5].speed_rpm,80*3591/187);
//		setMotoSpeed(&hcan1,pid_spd[4].pos_out,
//		                    pid_spd[5].pos_out,
//		                    0,
//		                    0,IDMARK_FIVE_EIGHT);//放下后脚
//	 
//	 
//	 mode=NORMAL_MODE;//退出下岛模式，进入一般模式
// }
//}

void giveBulletTaskFunction(void const * argument){
  for(;;){
	if(mode==GIVEBULLET){
		while(actionSignal!='o'){
			osDelay(500);
			HAL_UART_Transmit(&huart6,&mode,1,50);//发送任务信号
			HAL_UART_Receive(&huart6,&actionSignal,1,50);
     }
		actionSignal='f';//将任务完成信号置假
	 }
  }
}



