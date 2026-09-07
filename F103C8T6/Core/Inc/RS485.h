#ifndef  RS485_H
#define  RS485_H
#include "main.h"
#include "stm32f1xx_hal.h"


//硬件引脚规划
#define   RS485_Port         GPIOA
#define   RS485_pin          GPIO_PIN_1
#define   RS485_TX_EN        HAL_GPIO_WritePin(RS485_Port, RS485_pin, GPIO_PIN_SET);      //高电平发送
#define   RS485_RX_EN        HAL_GPIO_WritePin(RS485_Port, RS485_pin, GPIO_PIN_RESET);    //低电平接收

//数据包信息
#define  HEAD           0xAA




#define  MY_ID          Slave1_ID
#define  MASTER_ID      0x55
#define  Slave1_ID      0x01
#define  Slave2_ID      0x02
#define  Slave3_ID      0x03
#define  Slave4_ID      0x04
#define  Slave_all      0x05

extern  float target_velocity_1[1] ;
extern  float target_velocity_2[1] ;
extern  float target_velocity_3[1] ;
extern  float target_velocity_4[1] ;
extern  volatile float   target_angle_leftleg [3]  ;
extern  volatile float   target_angle_RIGHTleg [3] ;
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





void  RS485_SendBytes(  uint16_t *buf, uint16_t len, int  count_SET )  ;
void  Slave_SEND_DATA(uint16_t ID, float* float_arr, int size)  ;
void  test_crc8   (void) ;
uint8_t crc8_calculate  (const uint8_t *data, size_t len) ;
void MASTER_SEND_DATA (uint16_t ID , float* float_arr, int size);
void clear_ACK (void );
void Analyze_RS485_dataAnalyze_RS485_data (void);
void clear_ACK (void );
void  Analyze_ACKfrme_data  ( uint8_t SEND_ID1  , uint8_t receive_ID2 );
void Analyze_RS485_data (void);
void  send_ACKback ( uint8_t SEND_ID1  , uint8_t receive_ID2 );//收到了别人给我的报文
void RS485_SET_MOTEOR_SPEED ( float *data_1 , float *data_2 , float *data_3 , float *data_4);
void  send_ERRORback ( uint8_t SEND_ID1  , uint8_t receive_ID2 );
void delay_us(uint32_t us);
#endif


