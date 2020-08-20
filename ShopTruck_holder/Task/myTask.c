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
 * 2020-08-02                       加入云台视角运动函数 
 * 2020-08-15                       完成取弹电机的角度稳定控制，并且加入斜波函数防止电流过大
 */
#include "myTask.h"

int take_finish = 0;
int rescueFinish = 0;
int set_speed[2],set_angel;
int angle_set_user=0;
int control=1;
int val = 0;

uint8_t send_masge[9]={'0','0','0','0','0','0','0'};

uint8_t RXmasge = 0;
uint8_t TXmasge = 0;

int receive_masge[4];	

/**
    * @brief  取弹任务函数
    * @note   完成取弹的任务，包括定位，取弹,取弹药的部分逻辑要修改一下
    * @author 占建
    * @param  None
    * @retval 
    */
void takeBullteTaskFunction(void const * argument)
{
	for(;;)
	{
		HAL_UART_Receive_IT(&huart6,&RXmasge,1);
		deal_masge_get(RXmasge);
		if((receive_masge[3]==GETBULLET_1_MODE || receive_masge[3]==GETBULLET_10_MODE || receive_masge[3]==GETBULLET_4_MODE)&& receive_masge[2] == 1)
			{
				while(direction!=0)//当未定位到时就继续定位
					{
						direction=jugement();//定位
						if(direction==0)//两边没有检测到，为定位准确，取弹
							{
							/*速度清零防止初始电机乱转*/	
								set_speed[0] = 0;
								pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,set_speed[0]);
								setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																		 0,
																     0,
																     0,IDMARK_ONE_FOUR);
							/*取走中间弹药*/		 
								take(-180);	//爪子完成取弹
	
							/*取走中心弹药*/
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);//推到前面					
								take(-180);	//爪子完成取弹
										
							/*取走两侧弹药*/				
								for(int i=0;i<200;i++)
								{
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);//推到右边									
									pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);
									setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																			 0,
																			 0,
																			 0,IDMARK_ONE_FOUR);
									osDelay(1);
								}
							  take(-180);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);//回到原位
									 
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);//推到左边
								for(int i=0;i<200;i++)
								{						
									pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);
									setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																			 0,
																			 0,
																			 0,IDMARK_ONE_FOUR);
									osDelay(1);
								}
							  take(-180);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);//回到原位
								
							  take(-180);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);//回到原位
									
								for(int i =0;i<1000;i++)
								{
									pid_calc(&pid_spd[0], moto_chassis[0].total_angle,0);//定位回归原位置
									pid_calc(&pid_spd[4], moto_chassis[0].speed_rpm, pid_spd[0].pos_out);
									setMotoSpeed(&hcan1,pid_spd[4].pos_out,0,0,0,IDMARK_ONE_FOUR);
                  osDelay(1);									
								}
                take_finish =1;
                TXmasge=deal_masge_put(3,0,take_finish,0);								
								HAL_UART_Transmit(&huart6,&TXmasge,1,10);//把相关的信息传回底盘开发板
							
							/*将临时储弹仓里的弹药放到储弹仓里*/	
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				        osDelay(1000);
			         	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
								osDelay(1);
			        }

						if(direction==1)//01	
				     {
					   /*该情况为向右移动*/
							 set_speed[0] = 4000;
							 set_angel = 0;
							 pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,set_speed[0]);
							 setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																			 0,
																			 0,
																			 0,IDMARK_ONE_FOUR);
			       }			 

						if(direction==2)//10
						 {
							/*该情况为向右移动*/
							 set_speed[0] = -4000;
							 set_angel = 0;
							 pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,set_speed[0]);
							 setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																		0,
																		0,
																		0,IDMARK_ONE_FOUR);
						 }
		  }
	    direction = 3;//当一次取弹结束后，复位重新开始
			}
			else
			{
				take_finish =0;
        TXmasge=deal_masge_put(4,1,take_finish,1);								
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);//把相关的信息传回底盘开发板
			}
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//高电平，爪子打开
		osDelay(100);
	}	
}

/**
    * @brief  实现取弹的定位功能模块
    * @note   利用两个激光定位弹药箱的位置实现取弹定位
    * @author 占建
    * @param  None
    * @retval int 表示方向的移动
    */
int jugement(void)
	{
	  int direction=0;
	  if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7)==SET)
			direction=direction|2;  //2 = 10    三位二进制的各个位：0表示未检测到  1表示检测到
		if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_2)==SET)	
			direction=direction|1;  //1 = 01
		
		return direction;
  }


/**


    * @brief  实现爪子抓取弹药箱的操作
    * @note   目前实现了十字架取单操作，剩余一字取弹和四角取弹
    * @author 占建
    * @param  int :angel  设置角度值
    * @retval 
    */
void take(int angle )
	{ 
		//set_angel = angle*3591/187/360*8191*3.7;//由于设置了减速比和角度到数值的映射，所以角度设置要进行转换
		set_angel = angle*(1.0*C_1_2/C_1_1+1.0)*(1.0*C_2_2/C_2_1*1.0+1.0)*(1.0*C_3_2/C_3_1*1.0+1.0)/360.0*8192.0;//修改后的爪子，机械减速比，加小数是为了保留精度
							
	 /*PID延时，在电机需要稳定的时候，如果加入延时，就必须要在延时中不断的调节PID以达到稳固的效果*/ 
    for(int i=0;i<1000;i++)//延时1000毫秒使得爪子到达指定位置
    	{
				/*利用斜波函数进行输出，防止电流冲击太大*/
				if(speed_ramp_completed(&user_ramp))//完成后将会重置斜波函数
				speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
				val = speed_ramp_calc(&user_ramp);//作为PID的输出
				
				pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);//取弹时需要稳住定位电机
				pid_calc(&pid_spd[1], moto_chassis[1].total_angle,val);//通过角度环PID设置角度的增长	
				pid_calc(&pid_spd[3], moto_chassis[1].speed_rpm, pid_spd[1].pos_out);//将角度环的输出设定为速度环的输入，从而达到正确的角度控制
				setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
													   pid_spd[3].pos_out,
													   0,
													   0,IDMARK_ONE_FOUR);
				osDelay(1);
	    }
			
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);//弹出弹药箱
		for(int i=0;i<200;i++)//给予气缸通气时间
		  {
				pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);
				pid_calc(&pid_spd[1], moto_chassis[1].total_angle,set_angel);//前面理论上已经达到所需输出值，所以这里直接输入即可，不需要斜波
				pid_calc(&pid_spd[3], moto_chassis[1].speed_rpm, pid_spd[1].pos_out);
				setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
													   pid_spd[3].pos_out,
													   0,
													   0,IDMARK_ONE_FOUR);
				osDelay(1);
	  	}
			
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);//收回弹箱夹
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //低电平，爪子关闭
			
		for(int i=0;i<200;i++)
		  {
				pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);				
				pid_calc(&pid_spd[1], moto_chassis[1].total_angle,set_angel);
				pid_calc(&pid_spd[3], moto_chassis[1].speed_rpm, pid_spd[1].pos_out);
				setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
													   pid_spd[3].pos_out,
													   0,
													   0,IDMARK_ONE_FOUR);
				osDelay(1);
	  	}
			
		set_angel=-angle*(1.0*C_1_2/C_1_1+1.0)*(1.0*C_2_2/C_2_1*1.0+1.0)*(1.0*C_3_2/C_3_1*1.0+1.0)/360.0*8192.0; //回拉，倒弹丸
    for(int i=0;i<1200;i++)
	   {
				if(speed_ramp_completed(&user_ramp))
				speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
				val = speed_ramp_calc(&user_ramp);
				
				pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);				
				pid_calc(&pid_takeBullte[0], moto_chassis[1].total_angle,val);//由于回拉的力度需要很大，所以另外设置一套PID用于拉取弹药箱
				pid_calc(&pid_takeBullte[1], moto_chassis[1].speed_rpm, pid_takeBullte[0].pos_out);
				setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
													   pid_takeBullte[1].pos_out,
													   0,
													   0,IDMARK_ONE_FOUR);
				osDelay(1);
	   }
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//高电平，爪子打开		
	  
}


/**
    * @brief  云台任务函数
    * @note   接收底盘传来得到信息进行云台视角运动
    * @author 占建
    * @param  None
    * @retval 
    */

void holderTaskFunction(void const * argument)
	{
	  for(;;){
//			//遥控器数据解读部分截取
//			int x = (receive_masge[1] >> 6 | receive_masge[2] << 2 | receive_masge[3] << 10) & 0x07FF;
//       x -= 1024;
//      int y = (receive_masge[3] >> 1 | receive_masge[4] << 7) & 0x07FF;
//      y -= 1024;
//			
//			//转换为合适数值范围0-1000
//			x = x / 660 * 1000;
//		  y = -y / 660 *1000;
//			
//		  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,x);//云台左右视角
//      HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,y);//云台俯仰角			
//			
//		  osDelay(10);
		}
	
 }


/**
    * @brief  补弹任务实现
    * @note   将储弹仓的弹丸补给步兵和英雄车，还未完成小弹丸的补给操作
    * @author 占建
    * @param  None
    * @retval 
    */
void giveBullteTaskFunction(void const * argument)
 {
	for(;;)
	 {
			HAL_UART_Receive_IT(&huart6,&RXmasge,1);
		  deal_masge_get(RXmasge);
			if(receive_masge[3]==GIVEBULLET&&receive_masge[2]==1)//进入补弹模式并且抬起了电机
			{		
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);//打开供弹仓
				osDelay(3000);//三秒供弹
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);//关闭供弹仓
				
        take_finish = 1;				
        TXmasge=deal_masge_put(0,0,take_finish,0);								
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);//把相关的信息传回底盘开发板		
			}
			else
			{
//				take_finish =0;
//        TXmasge=deal_masge_put(0,0,take_finish,0);								
//				HAL_UART_Transmit(&huart6,&TXmasge,1,10);//把相关的信息传回底盘开发板
			}					
	 }
 }


/**
    * @brief  实现救援方案的救援爪部分
    * @note   代码中PWM参数值需要实际测试设定，确保抓稳，伸出，救援卡的电机控制待定
    *         TIM8设置的频率是50Hz,自动重载值是2000.由于左右舵机是类似为镜像对称关系，所以输出的占空比是相反的
    * @author 占建
    * @param  None
    * @retval 
    */
void rescueTaskeFunction(void const * argument)
{
		for(;;)
	{
		 HAL_UART_Receive_IT(&huart6,&RXmasge,1);
		 deal_masge_get(RXmasge);     
		 if(receive_masge[3] == 7 || receive_masge[3] == 9)//救援爪模式或者救援卡模式，两者都需要爪子闭合
		  {
			  /*依据机械结构要求，需要救援爪输出三个位置，所以有着这个标志判断*/    
				if(rescueFinish == 0)//救援得保证一次成功，否则得二次进入
				{
					  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		       	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,1500);//参数根据实际调节获得，右爪子
				    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		       	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,1700);//左爪子
						osDelay(2000);//等待两秒进入二阶抓取，使得抓取稳定
			    	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		       	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,500);
				    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		       	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,1100);
				   	rescueFinish = 1;
				    osDelay(2000);
				}
			 }
      else
		    {
          HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,2410);
					HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,2400);
          rescueFinish = 0;//清零标志位
		    }
		    osDelay(1);
	}
}

