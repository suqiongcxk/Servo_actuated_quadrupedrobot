#ifndef  RS485_H
#define  RS485_H
#include "main.h"
#include "stm32h7xx_hal.h"


//硬件引脚规划
#define   RS485_Port         GPIOB
#define   RS485_pin          GPIO_PIN_2
#define   RS485_TX_EN        HAL_GPIO_WritePin(RS485_Port, RS485_pin, GPIO_PIN_SET);      //高电平发送
#define   RS485_RX_EN        HAL_GPIO_WritePin(RS485_Port, RS485_pin, GPIO_PIN_RESET);    //低电平接收

//数据包信息
#define  HEAD           0xAA




#define  MY_ID          MASTER_ID
#define  MASTER_ID      0x55
#define  Slave1_ID      0x01
#define  Slave2_ID      0x02
#define  Slave3_ID      0x03
#define  Slave4_ID      0x04
#define  Slave_all      0x05

extern  float target_angle_1[6] ;
extern  float target_angle_2[6] ;
extern  float target_angle_3[6] ;
extern  float target_angle_4[6] ;
extern int  Slave1_ACK_ERR_CONT ;
extern int  Slave2_ACK_ERR_CONT ;
extern int  Slave3_ACK_ERR_CONT ;
extern int  Slave4_ACK_ERR_CONT ;



#define  END            0xBB







//报文功能   Message Function 
#define      DATA_Frame            		0x01
#define      Command_Frame         		0x02
#define      ACK_Frame             		0x03      //发送应答信号
#define      VElocity             		0x04      //获得数据指令
#define      wait_ack              		0x77
#define      get_ack               		0x88 
#define      ERROR_Frame           		0x99 





/* len_bytes is the byte count, e.g. sizeof the transmit array. */
void  RS485_SendBytes(  uint16_t *buf, uint16_t len_bytes, int  count_SET )  ;
void  Slave_SEND_DATA(uint16_t ID, float* float_arr, int size)  ;
void  test_crc8   (void) ;
uint8_t crc8_calculate  (const uint8_t *data, size_t len) ;
void MASTER_SEND_Frame(uint16_t ID, float *float_arr, int size  ,uint16_t frame_type);
void clear_ACK (void );
void clear_ACK (void );
void  Analyze_ACKfrme_data  ( uint8_t SEND_ID1  , uint8_t receive_ID2 );
void Analyze_RS485_data (void);
void  send_ACKback ( uint8_t SEND_ID1  , uint8_t receive_ID2 );//收到了别人给我的报文
void  send_ERRORback ( uint8_t SEND_ID1  , uint8_t receive_ID2 );
void delay_us(uint32_t us);
void RS485_TXANGLE(void);
#endif

