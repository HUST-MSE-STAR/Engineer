/* 
 * MYTASK.C - The C file of the MYTASK.h
 * NOTE: 
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes             mail
 * 2020-07-04     ElonJoker         rebuild version   2649853081@qq.com
 * 2020-07-10                       修改救援方案      
 * 2020-07-26                       测试重构后的CAN   
 * 2020-07-27                       加入PID           
 * 2020-07-29                       强制调用回调函数，暂时解决CAN接收中断一次的毛病
 * 2020-07-31                       解决了CAN的问题，版本较稳定 
 * 2020-08-04                       解决了CAN的中断问题，不需要强制调用回调函数
 * 2020-08-15                       整理代码规范，将初始化参数作独立文件
 */
#include "myTask.h"

uint8_t RXmasge = 0;
uint8_t TXmasge = 0;

int receive_masge[4];
int val = 0;
int rescue_finish = 0;
int up_finish = 0;

/**
    * @brief  底盘移动任务功能代码
    * @note   执行移动任务
    * @author 占建
    * @param  None
    * @retval None
    */
		
void moveTaskFunction(void const * argument)
{	
	/* Infinite loop */	
		  for(;;)
		{
	/*对四个轮子的PID 进行调节*/
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
									         pid_spd[1].pos_out,
									         pid_spd[2].pos_out,
									         pid_spd[3].pos_out,IDMARK_ONE_FOUR);			
	/*通过按键可以设置电机的基础马力(速度)配置*/
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
	/*获取遥控器的前后，左右控制器的输入数据*/
		vx = (float)rc.ch2 / 660 * 800;//遥控器通道值映射
		vy = -(float)rc.ch1 / 660 *800;
		wz = -rc.wheel*5.2;//这个5.2我理解错误了，可以自行设定，它决定底盘旋转的速度大小
	/*进入取弹模式则不移动*/
   if(mode==4 || mode==5|| mode==6)	
	 {
			set_spd[0]=set_spd[1]=set_spd[2]=set_spd[3]=0;
	 }
	 else
	 {
		/*麦克纳姆轮速度分解并设置电机速度*/
			set_spd[0] = (-vy - vx + wz)*(key_cnt+1);    
			set_spd[1] = (-vy + vx + wz)*(key_cnt+1);
			set_spd[2] = ( vy + vx + wz)*(key_cnt+1);
			set_spd[3] = ( vy - vx + wz)*(key_cnt+1);	
	 }
	 osDelay(1);//加入延时，使程序能够准确执行，过长或过短都会导致电机PID不稳定
   }	
}


/**
    * @brief  实现救援方案
    * @note   代码中PWM参数值需要实际测试设定，确保抓稳，伸出，救援卡的电机控制待定
    *         TIM8设置的频率是50Hz,自动重载值是20000.由于左右舵机是类似为镜像对称关系，要注意数值变化关系
    * @author 占建
    * @param  None
    * @retval 
    */
void rescueTaskeFunction(void const * argument)
{
	for(;;)
	{
		 if(mode==7 || mode==9)
		 {			
				if(mode==9)
				{
           if(!rescue_finish) //正转，伸出救援卡
					{								
						 pidDelay(10000,4,1250);
						 rescue_finish = 1;
					}
          else  //反转，收回救援卡
					{								
					 pidDelay(10000,4,0);
					 rescue_finish = 0;
					}
				}
			}
      else
		  {
				pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm,0);
				pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm,0);				
				setMotoSpeed(&hcan1,pid_spd[4].pos_out,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
		  }
		  osDelay(1);
	}
}

/**
    * @brief  云台和底盘通信任务实现函数
    * @note   执行通信任务
    * @author 占建
    * @param  None
    * @retval None
    */
void msgSendTaskFunction(void const * argument) 
{
	for(;;)
	{
		//判断此时的遥控器的模式
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_UP))//普通模式 右1 左1
			mode=NORMAL_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_MID))//普通模式下，补弹 右1 左2
	    mode=GIVEBULLET;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_DOWN))//普通模式下，登岛 右1 左3  暂空
			mode=CLIMB_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下，一排 右2  左1
			mode=GETBULLET_1_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_MID)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下，十字 右2  左2
		   mode=GETBULLET_10_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_DOWN)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下,四角 右2  左3
		   mode=GETBULLET_4_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_DOWN))//进入救援模式 右3 左1
			mode=RESCUE_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_MID))//退出救援模式，拨弹 右3 左2  暂空
		   mode=OPEN_MODE;
    
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_DOWN))//救援卡模式，拨弹 右3 左3
		   mode=RESCUE_CAR_MODE;		
		
		/*发送此时的模式和云台数据，所有板间通信均由此任务施行*/
    TXmasge=deal_masge_put(mode,up_finish,0,0);
		HAL_UART_Transmit(&huart6,&TXmasge,1,10);
		osDelay(1);
	}
	
}


/**
    * @brief  取弹辅助任务
    * @note   用于取弹机构抬升
    * @author 占建
    * @param  None
    * @retval 
    */
void upTaskeFunction(void const * argument)
	{
		for(;;)
		{	
			HAL_UART_Receive_IT(&huart6,&RXmasge,1);
			deal_masge_get(RXmasge);     
			if(mode == GETBULLET_1_MODE || mode == GETBULLET_4_MODE || mode == GETBULLET_10_MODE )
			{
//        /*利用角度抬升不太行，PID过小会抬不起来，太大会抖动*/				
//				if(receive_masge[0] == '0')
//					{
//						pidDelay(3000,5,-1260);//未完成取弹，就将取弹机构升起
//						send_masge[6] = '1';//告诉云台，取弹机构抬升完毕
//					}
//				else
//					{
//						pidDelay(3000,5,0);//完成取弹，就将取弹机构降下来
//						send_masge[6] = '0';//信息置零	
//					}				
          if(up_finish == 0)
					{
						for(int i = 0;i<4000;i++)
						{
					    pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, 7000);
					    setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
							osDelay(1);
						}
						up_finish = 1;//抬升完成
						/*将抬升完成的信号发送给云台，开始取弹*/
						TXmasge=deal_masge_put(mode,up_finish,0,0);
						HAL_UART_Transmit(&huart6,&TXmasge,1,10);						
					}
					
					HAL_UART_Receive_IT(&huart6,&RXmasge,1);
					deal_masge_get(RXmasge);
					if(receive_masge[2] == 0)//未完成取弹就继续保持电机稳定
					{
						for(int i = 0;i<2000;i++)
						{
					    pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, 0);
					    setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
							osDelay(1);
						}
					}
          else
					{
						for(int i = 0;i<3000;i++)
						{
							 pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, -6000);
							 setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
							 osDelay(1);
						}						
					}					
				osDelay(1);				
			}
			else
			{
       /*各类数据清零归位*/
        up_finish = 0;
				TXmasge=deal_masge_put(mode,up_finish,0,0);
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);						
			}				
		}
  }


/**
    * @brief  用于不准确延时，稳定电机
    * @note   电机的稳定需要pid,过长的延时会导致不稳定，所以以此函数解决问题
	            该函数主要用于控制角度的延时，效果很不错
    * @author 占建
    * @param  int tim_ms      大致延时时间
    *　　　　 int id          电机ID
    *         int set_angel   电机角度
　　×
    * @retval 
    */
void pidDelay(int tim_ms,int id,int set_angel)
	{
		for(int i =0; i<tim_ms;i++)
		{
			switch(id)
			{
				case 4:
					set_angel = set_angel*36*8191/360;//救援卡（2006）的减速比
				/*利用斜波函数进行输出，防止电流冲击太大*/
					if(speed_ramp_completed(&user_ramp))//完成后将会重置斜波函数
					speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
					val = speed_ramp_calc(&user_ramp);//作为PID的输出
					
					pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm,0);//取弹时需要稳住定位电机
					pid_calc(&pid_spd[4], moto_chassis[4].total_angle,val);//通过角度环PID设置角度的增长	
					pid_calc(&pid_rescue[0], moto_chassis[4].speed_rpm, pid_spd[4].pos_out);//将角度环的输出设定为速度环的输入，从而达到正确的角度控制
					setMotoSpeed(&hcan1,pid_rescue[0].pos_out,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
				  break;
				case 5:
					set_angel = set_angel*3591/187*8191/360;//抬升电机（3508）的减速比
				/*利用斜波函数进行输出，防止电流冲击太大*/
					if(speed_ramp_completed(&user_ramp))//完成后将会重置斜波函数
					speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
					val = speed_ramp_calc(&user_ramp);//作为PID的输出
					
					pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm,0);//取弹时需要稳住定位电机
					pid_calc(&pid_spd[5], moto_chassis[5].total_angle,val);//通过角度环PID设置角度的增长	
					pid_calc(&pid_rescue[1], moto_chassis[5].speed_rpm, pid_spd[5].pos_out);//将角度环的输出设定为速度环的输入，从而达到正确的角度控制
					setMotoSpeed(&hcan1,pid_spd[4].pos_out,pid_rescue[1].pos_out,0,0,IDMARK_FIVE_EIGHT);
				  break;
			}
			osDelay(1);
		}
	
  }

	
	
/**
      * @brief  补弹的辅助任务
      * @note   将机构抬起来，用于补弹
      * @author 占建
      * @param  None
      * @retval 
      */	
void giveTaskeFunction(void const * argument)
{
	for(;;)
	{
		HAL_UART_Receive_IT(&huart6,&RXmasge,1);
		deal_masge_get(RXmasge); 
		if(mode == GIVEBULLET)
		{
			if(up_finish == 0)
			{
				for(int i = 0;i<3000;i++)
			  {
					pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, 7000);
					setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
					osDelay(1);
				}
				up_finish = '1';//抬升完成
				/*将抬升完成的信号发送给云台，开始补弹*/
				TXmasge=deal_masge_put(mode,up_finish,0,0);
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);							
			}
			
			HAL_UART_Receive_IT(&huart6,&RXmasge,1);
			deal_masge_get(RXmasge); 

			if(receive_masge[2] == 0)//未完成补弹就继续保持电机稳定
			{
				for(int i = 0;i<2000;i++)
				{
					pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, 0);
					setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
					osDelay(1);
				}
			}
			else//完成补弹后降下取弹机构
			{
				for(int i = 0;i<2000;i++)
				{
					pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, -6000);
					setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
					osDelay(1);
				}	
        /*各类数据清零归位*/
        up_finish = '0';
				TXmasge=deal_masge_put(mode,up_finish,0,0);
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);									
			}				
			osDelay(1);
		}
	}
	
}	


