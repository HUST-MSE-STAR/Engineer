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
 * 2020-07-10                       �޸ľ�Ԯ����      
 * 2020-07-26                       �����ع����CAN   
 * 2020-07-27                       ����PID           
 * 2020-07-29                       ǿ�Ƶ��ûص���������ʱ���CAN�����ж�һ�ε�ë��
 * 2020-07-31                       �����CAN�����⣬�汾���ȶ� 
 * 2020-08-04                       �����CAN���ж����⣬����Ҫǿ�Ƶ��ûص�����
 * 2020-08-15                       �������淶������ʼ�������������ļ�
 */
#include "myTask.h"

uint8_t RXmasge = 0;
uint8_t TXmasge = 0;

int receive_masge[4];
int val = 0;
int rescue_finish = 0;
int up_finish = 0;

/**
    * @brief  �����ƶ������ܴ���
    * @note   ִ���ƶ�����
    * @author ռ��
    * @param  None
    * @retval None
    */
		
void moveTaskFunction(void const * argument)
{	
	/* Infinite loop */	
		  for(;;)
		{
	/*���ĸ����ӵ�PID ���е���*/
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
									         pid_spd[1].pos_out,
									         pid_spd[2].pos_out,
									         pid_spd[3].pos_out,IDMARK_ONE_FOUR);			
	/*ͨ�������������õ���Ļ�������(�ٶ�)����*/
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
	/*��ȡң������ǰ�����ҿ���������������*/
		vx = (float)rc.ch2 / 660 * 800;//ң����ͨ��ֵӳ��
		vy = -(float)rc.ch1 / 660 *800;
		wz = -rc.wheel*5.2;//���5.2���������ˣ����������趨��������������ת���ٶȴ�С
	/*����ȡ��ģʽ���ƶ�*/
   if(mode==4 || mode==5|| mode==6)	
	 {
			set_spd[0]=set_spd[1]=set_spd[2]=set_spd[3]=0;
	 }
	 else
	 {
		/*�����ķ���ٶȷֽⲢ���õ���ٶ�*/
			set_spd[0] = (-vy - vx + wz)*(key_cnt+1);    
			set_spd[1] = (-vy + vx + wz)*(key_cnt+1);
			set_spd[2] = ( vy + vx + wz)*(key_cnt+1);
			set_spd[3] = ( vy - vx + wz)*(key_cnt+1);	
	 }
	 osDelay(1);//������ʱ��ʹ�����ܹ�׼ȷִ�У���������̶��ᵼ�µ��PID���ȶ�
   }	
}


/**
    * @brief  ʵ�־�Ԯ����
    * @note   ������PWM����ֵ��Ҫʵ�ʲ����趨��ȷ��ץ�ȣ��������Ԯ���ĵ�����ƴ���
    *         TIM8���õ�Ƶ����50Hz,�Զ�����ֵ��20000.�������Ҷ��������Ϊ����Գƹ�ϵ��Ҫע����ֵ�仯��ϵ
    * @author ռ��
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
           if(!rescue_finish) //��ת�������Ԯ��
					{								
						 pidDelay(10000,4,1250);
						 rescue_finish = 1;
					}
          else  //��ת���ջؾ�Ԯ��
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
    * @brief  ��̨�͵���ͨ������ʵ�ֺ���
    * @note   ִ��ͨ������
    * @author ռ��
    * @param  None
    * @retval None
    */
void msgSendTaskFunction(void const * argument) 
{
	for(;;)
	{
		//�жϴ�ʱ��ң������ģʽ
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_UP))//��ͨģʽ ��1 ��1
			mode=NORMAL_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_MID))//��ͨģʽ�£����� ��1 ��2
	    mode=GIVEBULLET;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_DOWN))//��ͨģʽ�£��ǵ� ��1 ��3  �ݿ�
			mode=CLIMB_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_MID))//ȡ��ģʽ�£�һ�� ��2  ��1
			mode=GETBULLET_1_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_MID)&&rc_device_get_state(&rc,RC_S2_MID))//ȡ��ģʽ�£�ʮ�� ��2  ��2
		   mode=GETBULLET_10_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_DOWN)&&rc_device_get_state(&rc,RC_S2_MID))//ȡ��ģʽ��,�Ľ� ��2  ��3
		   mode=GETBULLET_4_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_DOWN))//�����Ԯģʽ ��3 ��1
			mode=RESCUE_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_MID))//�˳���Ԯģʽ������ ��3 ��2  �ݿ�
		   mode=OPEN_MODE;
    
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_DOWN))//��Ԯ��ģʽ������ ��3 ��3
		   mode=RESCUE_CAR_MODE;		
		
		/*���ʹ�ʱ��ģʽ����̨���ݣ����а��ͨ�ž��ɴ�����ʩ��*/
    TXmasge=deal_masge_put(mode,up_finish,0,0);
		HAL_UART_Transmit(&huart6,&TXmasge,1,10);
		osDelay(1);
	}
	
}


/**
    * @brief  ȡ����������
    * @note   ����ȡ������̧��
    * @author ռ��
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
//        /*���ýǶ�̧����̫�У�PID��С��̧��������̫��ᶶ��*/				
//				if(receive_masge[0] == '0')
//					{
//						pidDelay(3000,5,-1260);//δ���ȡ�����ͽ�ȡ����������
//						send_masge[6] = '1';//������̨��ȡ������̧�����
//					}
//				else
//					{
//						pidDelay(3000,5,0);//���ȡ�����ͽ�ȡ������������
//						send_masge[6] = '0';//��Ϣ����	
//					}				
          if(up_finish == 0)
					{
						for(int i = 0;i<4000;i++)
						{
					    pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, 7000);
					    setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
							osDelay(1);
						}
						up_finish = 1;//̧�����
						/*��̧����ɵ��źŷ��͸���̨����ʼȡ��*/
						TXmasge=deal_masge_put(mode,up_finish,0,0);
						HAL_UART_Transmit(&huart6,&TXmasge,1,10);						
					}
					
					HAL_UART_Receive_IT(&huart6,&RXmasge,1);
					deal_masge_get(RXmasge);
					if(receive_masge[2] == 0)//δ���ȡ���ͼ������ֵ���ȶ�
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
       /*�������������λ*/
        up_finish = 0;
				TXmasge=deal_masge_put(mode,up_finish,0,0);
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);						
			}				
		}
  }


/**
    * @brief  ���ڲ�׼ȷ��ʱ���ȶ����
    * @note   ������ȶ���Ҫpid,��������ʱ�ᵼ�²��ȶ��������Դ˺����������
	            �ú�����Ҫ���ڿ��ƽǶȵ���ʱ��Ч���ܲ���
    * @author ռ��
    * @param  int tim_ms      ������ʱʱ��
    *�������� int id          ���ID
    *         int set_angel   ����Ƕ�
������
    * @retval 
    */
void pidDelay(int tim_ms,int id,int set_angel)
	{
		for(int i =0; i<tim_ms;i++)
		{
			switch(id)
			{
				case 4:
					set_angel = set_angel*36*8191/360;//��Ԯ����2006���ļ��ٱ�
				/*����б�����������������ֹ�������̫��*/
					if(speed_ramp_completed(&user_ramp))//��ɺ󽫻�����б������
					speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
					val = speed_ramp_calc(&user_ramp);//��ΪPID�����
					
					pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm,0);//ȡ��ʱ��Ҫ��ס��λ���
					pid_calc(&pid_spd[4], moto_chassis[4].total_angle,val);//ͨ���ǶȻ�PID���ýǶȵ�����	
					pid_calc(&pid_rescue[0], moto_chassis[4].speed_rpm, pid_spd[4].pos_out);//���ǶȻ�������趨Ϊ�ٶȻ������룬�Ӷ��ﵽ��ȷ�ĽǶȿ���
					setMotoSpeed(&hcan1,pid_rescue[0].pos_out,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
				  break;
				case 5:
					set_angel = set_angel*3591/187*8191/360;//̧�������3508���ļ��ٱ�
				/*����б�����������������ֹ�������̫��*/
					if(speed_ramp_completed(&user_ramp))//��ɺ󽫻�����б������
					speed_ramp_init(&user_ramp, set_angel,STEP_VAL);
					val = speed_ramp_calc(&user_ramp);//��ΪPID�����
					
					pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm,0);//ȡ��ʱ��Ҫ��ס��λ���
					pid_calc(&pid_spd[5], moto_chassis[5].total_angle,val);//ͨ���ǶȻ�PID���ýǶȵ�����	
					pid_calc(&pid_rescue[1], moto_chassis[5].speed_rpm, pid_spd[5].pos_out);//���ǶȻ�������趨Ϊ�ٶȻ������룬�Ӷ��ﵽ��ȷ�ĽǶȿ���
					setMotoSpeed(&hcan1,pid_spd[4].pos_out,pid_rescue[1].pos_out,0,0,IDMARK_FIVE_EIGHT);
				  break;
			}
			osDelay(1);
		}
	
  }

	
	
/**
      * @brief  �����ĸ�������
      * @note   ������̧���������ڲ���
      * @author ռ��
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
				up_finish = '1';//̧�����
				/*��̧����ɵ��źŷ��͸���̨����ʼ����*/
				TXmasge=deal_masge_put(mode,up_finish,0,0);
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);							
			}
			
			HAL_UART_Receive_IT(&huart6,&RXmasge,1);
			deal_masge_get(RXmasge); 

			if(receive_masge[2] == 0)//δ��ɲ����ͼ������ֵ���ȶ�
			{
				for(int i = 0;i<2000;i++)
				{
					pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, 0);
					setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
					osDelay(1);
				}
			}
			else//��ɲ�������ȡ������
			{
				for(int i = 0;i<2000;i++)
				{
					pid_calc(&pid_spd[5], moto_chassis[5].speed_rpm, -6000);
					setMotoSpeed(&hcan1,0,pid_spd[5].pos_out,0,0,IDMARK_FIVE_EIGHT);
					osDelay(1);
				}	
        /*�������������λ*/
        up_finish = '0';
				TXmasge=deal_masge_put(mode,up_finish,0,0);
				HAL_UART_Transmit(&huart6,&TXmasge,1,10);									
			}				
			osDelay(1);
		}
	}
	
}	


