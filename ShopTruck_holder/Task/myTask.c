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
 * 2020-08-02                       ������̨�ӽ��˶����� 
 * 2020-08-15                       ���ȡ������ĽǶ��ȶ����ƣ����Ҽ���б��������ֹ��������
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
    * @brief  ȡ��������
    * @note   ���ȡ�������񣬰�����λ��ȡ��,ȡ��ҩ�Ĳ����߼�Ҫ�޸�һ��
    * @author ռ��
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
				while(direction!=0)//��δ��λ��ʱ�ͼ�����λ
					{
						direction=jugement();//��λ
						if(direction==0)//����û�м�⵽��Ϊ��λ׼ȷ��ȡ��
							{
							/*�ٶ������ֹ��ʼ�����ת*/	
								set_speed[0] = 0;
								pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,set_speed[0]);
								setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																		 0,
																     0,
																     0,IDMARK_ONE_FOUR);
							/*ȡ���м䵯ҩ*/		 
								take(-180);	//צ�����ȡ��
	
							/*ȡ�����ĵ�ҩ*/
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_RESET);//�Ƶ�ǰ��					
								take(-180);	//צ�����ȡ��
										
							/*ȡ�����൯ҩ*/				
								for(int i=0;i<200;i++)
								{
									HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_RESET);//�Ƶ��ұ�									
									pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);
									setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																			 0,
																			 0,
																			 0,IDMARK_ONE_FOUR);
									osDelay(1);
								}
							  take(-180);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,GPIO_PIN_SET);//�ص�ԭλ
									 
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);//�Ƶ����
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
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);//�ص�ԭλ
								
							  take(-180);
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,GPIO_PIN_SET);//�ص�ԭλ
									
								for(int i =0;i<1000;i++)
								{
									pid_calc(&pid_spd[0], moto_chassis[0].total_angle,0);//��λ�ع�ԭλ��
									pid_calc(&pid_spd[4], moto_chassis[0].speed_rpm, pid_spd[0].pos_out);
									setMotoSpeed(&hcan1,pid_spd[4].pos_out,0,0,0,IDMARK_ONE_FOUR);
                  osDelay(1);									
								}
                take_finish =1;
                TXmasge=deal_masge_put(3,0,take_finish,0);								
								HAL_UART_Transmit(&huart6,&TXmasge,1,10);//����ص���Ϣ���ص��̿�����
							
							/*����ʱ��������ĵ�ҩ�ŵ���������*/	
								HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_SET);
				        osDelay(1000);
			         	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,GPIO_PIN_RESET);
								osDelay(1);
			        }

						if(direction==1)//01	
				     {
					   /*�����Ϊ�����ƶ�*/
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
							/*�����Ϊ�����ƶ�*/
							 set_speed[0] = -4000;
							 set_angel = 0;
							 pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,set_speed[0]);
							 setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
																		0,
																		0,
																		0,IDMARK_ONE_FOUR);
						 }
		  }
	    direction = 3;//��һ��ȡ�������󣬸�λ���¿�ʼ
			}
			else
			{
				take_finish =0;
        TXmasge=deal_masge_put(4,1,take_finish,1);								
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);//����ص���Ϣ���ص��̿�����
			}
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//�ߵ�ƽ��צ�Ӵ�
		osDelay(100);
	}	
}

/**
    * @brief  ʵ��ȡ���Ķ�λ����ģ��
    * @note   �����������ⶨλ��ҩ���λ��ʵ��ȡ����λ
    * @author ռ��
    * @param  None
    * @retval int ��ʾ������ƶ�
    */
int jugement(void)
	{
	  int direction=0;
	  if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_7)==SET)
			direction=direction|2;  //2 = 10    ��λ�����Ƶĸ���λ��0��ʾδ��⵽  1��ʾ��⵽
		if(HAL_GPIO_ReadPin(GPIOI,GPIO_PIN_2)==SET)	
			direction=direction|1;  //1 = 01
		
		return direction;
  }


/**


    * @brief  ʵ��צ��ץȡ��ҩ��Ĳ���
    * @note   Ŀǰʵ����ʮ�ּ�ȡ��������ʣ��һ��ȡ�����Ľ�ȡ��
    * @author ռ��
    * @param  int :angel  ���ýǶ�ֵ
    * @retval 
    */
void take(int angle )
	{ 
		//set_angel = angle*3591/187/360*8191*3.7;//���������˼��ٱȺͽǶȵ���ֵ��ӳ�䣬���ԽǶ�����Ҫ����ת��
		set_angel = angle*(1.0*C_1_2/C_1_1+1.0)*(1.0*C_2_2/C_2_1*1.0+1.0)*(1.0*C_3_2/C_3_1*1.0+1.0)/360.0*8192.0;//�޸ĺ��צ�ӣ���е���ٱȣ���С����Ϊ�˱�������
							
	 /*PID��ʱ���ڵ����Ҫ�ȶ���ʱ�����������ʱ���ͱ���Ҫ����ʱ�в��ϵĵ���PID�Դﵽ�ȹ̵�Ч��*/ 
    for(int i=0;i<1000;i++)//��ʱ1000����ʹ��צ�ӵ���ָ��λ��
    	{
				/*����б�����������������ֹ�������̫��*/
				if(speed_ramp_completed(&user_ramp))//��ɺ󽫻�����б������
				speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
				val = speed_ramp_calc(&user_ramp);//��ΪPID�����
				
				pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);//ȡ��ʱ��Ҫ��ס��λ���
				pid_calc(&pid_spd[1], moto_chassis[1].total_angle,val);//ͨ���ǶȻ�PID���ýǶȵ�����	
				pid_calc(&pid_spd[3], moto_chassis[1].speed_rpm, pid_spd[1].pos_out);//���ǶȻ�������趨Ϊ�ٶȻ������룬�Ӷ��ﵽ��ȷ�ĽǶȿ���
				setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
													   pid_spd[3].pos_out,
													   0,
													   0,IDMARK_ONE_FOUR);
				osDelay(1);
	    }
			
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_RESET);//������ҩ��
		for(int i=0;i<200;i++)//��������ͨ��ʱ��
		  {
				pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);
				pid_calc(&pid_spd[1], moto_chassis[1].total_angle,set_angel);//ǰ���������Ѿ��ﵽ�������ֵ����������ֱ�����뼴�ɣ�����Ҫб��
				pid_calc(&pid_spd[3], moto_chassis[1].speed_rpm, pid_spd[1].pos_out);
				setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
													   pid_spd[3].pos_out,
													   0,
													   0,IDMARK_ONE_FOUR);
				osDelay(1);
	  	}
			
		HAL_GPIO_WritePin(GPIOH,GPIO_PIN_12,GPIO_PIN_SET);//�ջص����
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_RESET); //�͵�ƽ��צ�ӹر�
			
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
			
		set_angel=-angle*(1.0*C_1_2/C_1_1+1.0)*(1.0*C_2_2/C_2_1*1.0+1.0)*(1.0*C_3_2/C_3_1*1.0+1.0)/360.0*8192.0; //������������
    for(int i=0;i<1200;i++)
	   {
				if(speed_ramp_completed(&user_ramp))
				speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
				val = speed_ramp_calc(&user_ramp);
				
				pid_calc(&pid_spd[0], moto_chassis[0].speed_rpm,0);				
				pid_calc(&pid_takeBullte[0], moto_chassis[1].total_angle,val);//���ڻ�����������Ҫ�ܴ�������������һ��PID������ȡ��ҩ��
				pid_calc(&pid_takeBullte[1], moto_chassis[1].speed_rpm, pid_takeBullte[0].pos_out);
				setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
													   pid_takeBullte[1].pos_out,
													   0,
													   0,IDMARK_ONE_FOUR);
				osDelay(1);
	   }
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,GPIO_PIN_SET);//�ߵ�ƽ��צ�Ӵ�		
	  
}


/**
    * @brief  ��̨������
    * @note   ���յ��̴����õ���Ϣ������̨�ӽ��˶�
    * @author ռ��
    * @param  None
    * @retval 
    */

void holderTaskFunction(void const * argument)
	{
	  for(;;){
//			//ң�������ݽ�����ֽ�ȡ
//			int x = (receive_masge[1] >> 6 | receive_masge[2] << 2 | receive_masge[3] << 10) & 0x07FF;
//       x -= 1024;
//      int y = (receive_masge[3] >> 1 | receive_masge[4] << 7) & 0x07FF;
//      y -= 1024;
//			
//			//ת��Ϊ������ֵ��Χ0-1000
//			x = x / 660 * 1000;
//		  y = -y / 660 *1000;
//			
//		  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,x);//��̨�����ӽ�
//      HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
//			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,y);//��̨������			
//			
//		  osDelay(10);
		}
	
 }


/**
    * @brief  ��������ʵ��
    * @note   �������ֵĵ��貹��������Ӣ�۳�����δ���С����Ĳ�������
    * @author ռ��
    * @param  None
    * @retval 
    */
void giveBullteTaskFunction(void const * argument)
 {
	for(;;)
	 {
			HAL_UART_Receive_IT(&huart6,&RXmasge,1);
		  deal_masge_get(RXmasge);
			if(receive_masge[3]==GIVEBULLET&&receive_masge[2]==1)//���벹��ģʽ����̧���˵��
			{		
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);//�򿪹�����
				osDelay(3000);//���빩��
				HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_RESET);//�رչ�����
				
        take_finish = 1;				
        TXmasge=deal_masge_put(0,0,take_finish,0);								
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);//����ص���Ϣ���ص��̿�����		
			}
			else
			{
//				take_finish =0;
//        TXmasge=deal_masge_put(0,0,take_finish,0);								
//				HAL_UART_Transmit(&huart6,&TXmasge,1,10);//����ص���Ϣ���ص��̿�����
			}					
	 }
 }


/**
    * @brief  ʵ�־�Ԯ�����ľ�Ԯצ����
    * @note   ������PWM����ֵ��Ҫʵ�ʲ����趨��ȷ��ץ�ȣ��������Ԯ���ĵ�����ƴ���
    *         TIM8���õ�Ƶ����50Hz,�Զ�����ֵ��2000.�������Ҷ��������Ϊ����Գƹ�ϵ�����������ռ�ձ����෴��
    * @author ռ��
    * @param  None
    * @retval 
    */
void rescueTaskeFunction(void const * argument)
{
		for(;;)
	{
		 HAL_UART_Receive_IT(&huart6,&RXmasge,1);
		 deal_masge_get(RXmasge);     
		 if(receive_masge[3] == 7 || receive_masge[3] == 9)//��Ԯצģʽ���߾�Ԯ��ģʽ�����߶���Ҫצ�ӱպ�
		  {
			  /*���ݻ�е�ṹҪ����Ҫ��Ԯצ�������λ�ã��������������־�ж�*/    
				if(rescueFinish == 0)//��Ԯ�ñ�֤һ�γɹ�������ö��ν���
				{
					  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
		       	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,1500);//��������ʵ�ʵ��ڻ�ã���צ��
				    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
		       	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,1700);//��צ��
						osDelay(2000);//�ȴ�����������ץȡ��ʹ��ץȡ�ȶ�
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
          rescueFinish = 0;//�����־λ
		    }
		    osDelay(1);
	}
}

