#ifndef __BSP_DBUS_H__
#define __BSP_DBUS_H__

#include "usart.h"

#define UART_RX_DMA_SIZE (1024)
#define DBUS_MAX_LEN     (50)
#define DBUS_BUFLEN      (18)
#define DBUS_HUART       huart1 /* for dji remote controler reciever */
/** 
  * @brief  remote control information
  */
#define RC_S1_UP2MID   (1 << 0u)
#define RC_S1_MID2UP   (1 << 1u)
#define RC_S1_DOWN2MID (1 << 2u)
#define RC_S1_MID2DOWN (1 << 3u)

#define RC_S2_UP2MID   (1 << 4u)
#define RC_S2_MID2UP   (1 << 5u)
#define RC_S2_DOWN2MID (1 << 6u)
#define RC_S2_MID2DOWN (1 << 7u)

#define RC_S1_UP       (1 << 8u)
#define RC_S1_MID      (1 << 9u)
#define RC_S1_DOWN     (1 << 10u)
#define RC_S2_UP       (1 << 11u)
#define RC_S2_MID      (1 << 12u)
#define RC_S2_DOWN     (1 << 13u)
	
typedef __packed struct
{
  /* rocker channel information */
  int16_t ch1;
  int16_t ch2;
  int16_t ch3;
  int16_t ch4;
	
  /* left and right lever information */
  uint8_t sw1;
  uint8_t sw2;
	
	uint8_t last_sw1;
  uint8_t last_sw2;
	
	uint16_t state;
	
	__packed struct 
  {
    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
  } mouse;
  /* keyboard key information */
  __packed union {
    uint16_t key_code;
    __packed struct
    {
      uint16_t W : 1;
      uint16_t S : 1;
      uint16_t A : 1;
      uint16_t D : 1;
      uint16_t SHIFT : 1;
      uint16_t CTRL : 1;
      uint16_t Q : 1;
      uint16_t E : 1;
      uint16_t R : 1;
      uint16_t F : 1;
			
      uint16_t G : 1;
      uint16_t Z : 1;
      uint16_t X : 1;
      uint16_t C : 1;
      uint16_t V : 1;
      uint16_t B : 1;
    } bit;
  } kb;
  int16_t wheel;
} rc_info_t;

extern rc_info_t rc;

void uart_receive_handler(UART_HandleTypeDef *huart);
void dbus_uart_init(void);
static void get_dr16_state(rc_info_t *rc_dev);
int8_t rc_device_get_state(rc_info_t *rc_dev, uint16_t state);
#endif

