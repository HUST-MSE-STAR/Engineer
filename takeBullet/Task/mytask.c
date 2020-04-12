#include "mytask.h"

/*
任务函数实现
*/

void takeBullteTaskFunction(void const * argument)
{
	for(;;){
	  userCanInit(&hcan1);
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);//此处有待解决！！！！！！
		//完成电机的转动配置
					pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm, set_speed);//一号为定位电机，主要设置转速
		      pid_calc(&pid_spd[1], moto_chassis[1].angle, set_angel);//二号为取弹电机,主要设置角度
			setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
						               pid_spd[1].pos_out,
									         0,
									         0);	
		
		HAL_UART_Receive(&huart6,masge,7,50);
		
		if(masge[5]==GETBULLET_1_MODE|masge[5]==GETBULLET_10_MODE|masge[5]==GETBULLET_4_MODE){
    direction=jugement();
     if(direction==2|direction==7){
			 	//中间检测到，两边没有，为定位准确，取弹
		   set_speed=0;
/*取走中间弹药*/		 
       take();//爪子取弹
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);//将弹药导入供弹箱
			 
	     //高电平，伸出爪子
		   HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);
			 
       take();
			 
/*取走两侧弹药*/				
       HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);//推到左边
			  take();
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);//回到原位
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);//将弹药导入供弹箱
			 
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);//推到右边
			  take();
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);//回到原位
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);//将弹药导入供弹箱
			 osDelay(2000);
/*将取弹机构降下来*/			 
      actionSignal='o';  //发送完成取弹的信号
			HAL_UART_Transmit(&huart6,&actionSignal,1,50);
		 }

		 if(direction==0|direction==1|direction==3)	{
			  //该三种情况为向右移动
		   set_speed=2000;
			 set_angel=0;
		 }			 

		 if(direction==4|direction==5|direction==6){
			   //该三种情况为向左移动
		   set_speed=2000;
			 set_angel=0;
		 }
	  	direction=0;//当一次判断结束后，复位重新开始
	}
	
}	
}


void giveBullteBullteTaskFunction(void const * argument){
  
	uint8_t signal;
	
	HAL_UART_Receive(&huart6,masge,7,50);//接收任务信号
	if(masge[5]==GIVEBULLTE){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);
		osDelay(3000);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);
		signal='o';
		HAL_UART_Transmit(&huart6,&signal,1,50);//发送任务完成信号
	}
}

void holderTaskFunction(void const * argument){
	for(;;){
	//开启PWM通道
	int x,y;//
		HAL_UART_Receive(&huart6,masge,7,50);
		   x=(masge[1]>> 1 | masge[2] << 7) & 0x07FF;
		   y=(masge[3]>> 1 | masge[4] << 7) & 0x07FF;
	    	x-=1024;//研究一下，貌似是原来测试RC的时候的问题
		    y-=1024;
		
    	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,x);//自动重载值为1000
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,y);
	}
}

/*
模块功能函数实现
*/

int jugement(void){
	  int direction=0;
	  if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==SET)
			direction=direction|4;  //4 = 100    三位二进制的各个位：0表示未检测到  1表示检测到
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_5)==SET)		
			direction=direction|2;  //2 = 010
		if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_6)==SET)	
			direction=direction|1;  //1 = 001
		
		return direction;
}

void take(void ){
				set_angel=80*3591/187;//由于设置了减速比，所以角度设置要转换 转80度
			 osDelay(500);
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET); //高电平，爪子关闭
			 set_angel=-80*3591/187; //回拉，倒弹丸
			 osDelay(500);
			 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET);//低电平，爪子打开，弹出弹药箱
}


