#include "RS485.h"
#include "main.h"
#include "usart.h"
#include <stdint.h>
#include <string.h>
#include "stdio.h"
#include "usart.h"
#include <stdint.h>
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
float velocity_arr[1] = {0};
float target_velocity_1[1] = {0};
float target_velocity_2[1] = {0};
float target_velocity_3[1] = {0};
float target_velocity_4[1] = {0};

volatile float  target_angle_leftleg [3]  ={0} ;
volatile float  target_angle_RIGHTleg [3] ={0} ;
//应答数据定义
int  Slave1_ACK   = 0;
int  Slave2_ACK   = 0;
int  Slave3_ACK   = 0;
int  Slave4_ACK   = 0;
int  Master_ACK   = 0;
int  Slave1_ACK_ERR_CONT  = 0;
int  Slave2_ACK_ERR_CONT  = 0;
int  Slave3_ACK_ERR_CONT  = 0;
int  Slave4_ACK_ERR_CONT  = 0;


int IQ_ERROR = 0;


//CRC表

static const uint8_t crc8_table[256] = {
    0x00, 0x07, 0x0E, 0x09, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D,
    0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D,
    0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD,
    0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD,
    0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA,
    0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A,
    0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0D, 0x0A,
    0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A,
    0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4,
    0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4,
    0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44,
    0x19, 0x1E, 0x17, 0x10, 0x05, 0x02, 0x0B, 0x0C, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34,
    0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63,
    0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x06, 0x01, 0x08, 0x0F, 0x1A, 0x1D, 0x14, 0x13,
    0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83,
    0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3
};




/**
 * @brief CRC-8 校验函数 (查表法)
 * @param data 待校验数据指针
 * @param len 数据长度(字节)
 * @return CRC8校验值
 * @note 多项式: x^8 + x^2 + x + 1 (0x07), 初始值: 0xFF
 */
uint8_t crc8_calculate(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;  // 初始值
    
    for (size_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}





void test_crc8(void) 
    {
    uint8_t test_data[] = {0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39};
    uint8_t crc = crc8_calculate(test_data, sizeof(test_data));
    
    // printf("CRC-8 (多项式0x07, 初始值0xFF) = 0x%02X\n", crc);   //理论值0xFB
    }


		
		
// 获取速度值数据帧 (支持float类型)
void Slave_SEND_DATA(uint16_t ID, float* float_arr, int size)  
{
    clear_ACK(); //发送前清空
    // 帧结构: [包头2] [ID信息2] [长度1] [数据(N*4)] [CRC8 1] [包尾1]
    // 每个float拆分为2个uint16_t, 共 N*4 字节
    
    int data_bytes = 4 * size;          // 数据部分字节数 (float占4字节)
    int total_words = 2 * size + 4;       // 总数组长度(包含帧头、长度、CRC、帧尾)
    
    uint16_t rs485_buf[total_words];
    uint8_t *byte_buf = (uint8_t*)rs485_buf;  // 用于CRC计算的指针
    
    // 填充帧头
    rs485_buf[0] = (HEAD      << 8) | DATA_Frame;    // 包头 + 功能码
    rs485_buf[1] = (MY_ID << 8) | ID;            // 主机ID + 从机ID
    
    // 填充数据长度 (字节数)
    rs485_buf[2] = data_bytes;
    
    // 填充数据: float → int32 → 拆分为两个uint16_t
    for (int i = 0; i < size; i++) {
        union {
            float f;
            uint32_t u32;
        } f2u = { .f = float_arr[i] };
        
        rs485_buf[3 + 2*i]     = (f2u.u32 >> 16) & 0xFFFF;  // 高16位
        rs485_buf[3 + 2*i + 1] = f2u.u32 & 0xFFFF;          // 低16位
    }
    
    // 计算CRC8 (覆盖帧头、长度、数据)
    int crc_length = 2 * (2 + 1 + 2*size);  // 前3个word + 数据部分
    uint8_t crc8 = crc8_calculate(byte_buf, crc_length);
    
    // 填充CRC8和帧尾
    rs485_buf[3 + 2*size] = (crc8   << 8) | END;
    
    
    RS485_SendBytes(rs485_buf, sizeof(rs485_buf),1);  //RS485发送
}


// 主机发送数据帧 (支持float类型)
void MASTER_SEND_DATA (uint16_t ID , float* float_arr, int size)
{

    // 帧结构: [包头2] [ID信息2] [长度1] [数据(N*4)] [CRC8 1] [包尾1]
    // 每个float拆分为2个uint16_t, 共 N*4 字节
    
    int data_bytes = 4 * size;          // 数据部分字节数 (float占4字节)
    int total_words = 2 * size + 4;       // 总数组长度(包含帧头、长度、CRC、帧尾)
    
    uint16_t rs485_buf[total_words];
    uint8_t *byte_buf = (uint8_t*)rs485_buf;  // 用于CRC计算的指针
    
    // 填充帧头
    rs485_buf[0] = (HEAD      << 8) | DATA_Frame;    // 包头 + 功能码
    rs485_buf[1] = (MASTER_ID << 8) | ID;            // 主机ID + 从机ID
    
    // 填充数据长度 (字节数)
    rs485_buf[2] = data_bytes;
    
    // 填充数据: float → int32 → 拆分为两个uint16_t
    for (int i = 0; i < size; i++) {
        union {
            float f;
            uint32_t u32;
        } f2u = { .f = float_arr[i] };
        
        rs485_buf[3 + 2*i]     = (f2u.u32 >> 16) & 0xFFFF;  // 高16位
        rs485_buf[3 + 2*i + 1] = f2u.u32 & 0xFFFF;          // 低16位
    }
    
    // 计算CRC8 (覆盖帧头、长度、数据)
    int crc_length = 2 * (2 + 1 + 2*size);  // 前3个word + 数据部分
    uint8_t crc8 = crc8_calculate(byte_buf, crc_length);
    
    // 填充CRC8和帧尾
    rs485_buf[3 + 2*size] = (crc8   << 8) | END;
    
    
    RS485_SendBytes(rs485_buf, sizeof(rs485_buf),1);  //RS485发送

}




//请求从机发送数据命令
void  MASTER_ENABLE_Slave_TX( uint16_t ID   ,uint16_t command )
{
    uint16_t rs485_buf[4];
    rs485_buf[0] = (HEAD      << 8) | Command_Frame;     // 包头 + 功能码
    rs485_buf[1] = (MASTER_ID << 8) | ID;                // 主机ID + 从机ID
    rs485_buf[2] = command;                               // 命令长度 1字节
    rs485_buf[3] = (0x00   << 8) | END;
    RS485_SendBytes(rs485_buf, sizeof(rs485_buf),0);  //RS485发送
}




/**
 * 精确的us级延时
 * @brief 使用循环计数实现微秒级延时
 * @param us: 延时的微秒数
 * @note 需要根据主频调整DELAY_US_COUNT
F103一个周期六1.4纳秒
 */
void delay_us(uint32_t us)
{
    // 计算循环次数
    // 每个循环大约需要9个时钟周期
    const uint32_t DELAY_US_COUNT = SystemCoreClock / 205000.0f / 9.0f;
    
    for(uint32_t i = 0; i < us; i++) {
        for(uint32_t j = 0; j < DELAY_US_COUNT; j++) {
            __asm__ volatile ("nop");
        }
    }
}

//485发送数据    数组地址，长度，是否需要应答信号，自动重传次数

void  RS485_SendBytes(  uint16_t *buf, uint16_t len, int  count_SET ) 
{

    RS485_TX_EN;
		delay_us(2);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *) buf, len);
    while (huart1.gState != HAL_UART_STATE_READY);//等待发送完成
    RS485_RX_EN;
		delay_us(2);
		//清零计数器
		TIM4->CNT = 0; 

		while (  TIM4->CNT  <= 100) // 100us内没有收到应答数据等待
	{
			if (RS485_flag == 1)
			{
			break;
			}
			taskYIELD();
	}	
	
//	printf("%d\n",RS485_flag);

	 if( RS485_flag == 0 )
	 {
    if(count_SET != 0   )  //需要重传并且没收到数据
    {
        for (int count = 0;count < count_SET ;count++)
        {
            if (RS485_flag == 1) return ;
            RS485_TX_EN;
						delay_us(2);
            HAL_UART_Transmit_DMA(&huart1, (uint8_t *) buf, len);
            while (huart1.gState != HAL_UART_STATE_READY){};//等待发送完成
             RS485_RX_EN;
						delay_us(2);
						//清零计数器
						TIM4->CNT = 0;
            while (  TIM4->CNT  <= 100) // 100us内没有收到应答数据等待
            {							  
                if (RS485_flag == 1){return ;}  
            }
        }
    }
	  }


 }


 //485接收数据处理
void Analyze_RS485_data (void)
{

    // 01 AA 01 55 10 
    // 00 00 00 00 00 
    // 00 00 00 00 00 
    // 00 00 00 00 00 
    // 00 00 BB 1A   数据帧例子      
	
	/*
	3	AA	55  1 	BB  命令帧
	*/
    if(RS485_flag)
    {
    uint8_t FRAME_type = 0x00;
    FRAME_type = RS485_dma_buffer[0]  ;    //获取报文类型
    uint8_t ID1  = RS485_dma_buffer[3] ;   //发送方ID
    uint8_t ID2  = RS485_dma_buffer[2] ;   //接收方ID

//		printf("%02X\n",ID2);

    if (ID2 != MY_ID ) {RS485_flag = 0;return;}
    
    if(FRAME_type == DATA_Frame)    //这是一个数据报文，让我看看是什么数据
    {
        if( RS485_dma_buffer[1]    == 0xAA )   //包头没问题
        {

        uint8_t data_length = RS485_dma_buffer[4] /4; //float数据个数，字节数
        
        float float_ARRY[data_length] ; 
        uint16_t  gao_16  =0 ;
        uint16_t  di_16   =0 ;
        for (int i =0; i < data_length ; i++)  //遍历每一个浮点数
        {
            gao_16 = RS485_dma_buffer[6+4*i+1]    <<8 |  RS485_dma_buffer[6+4*i ]  ;  //高16位 
            di_16 = ((uint16_t)RS485_dma_buffer[9 + 4*i] << 8)
                  | RS485_dma_buffer[8 + 4*i];  //低16位：数据区从字节6开始
            uint32_t combined = ((uint32_t)gao_16 << 16) | di_16;
            memcpy(&float_ARRY[i], &combined, sizeof(float_ARRY[i]));
        } 
        uint8_t crc_Frme = RS485_dma_buffer[5 + data_length *4 +2]   ; //CRC8校验值 
        uint8_t  crc_cal  = crc8_calculate( RS485_dma_buffer ,   6+ data_length*4) ; //计算CRC8校验值
        
        if(crc_Frme == crc_cal )  //CRC校验通过
        {
            //接收本次速度数据
            switch ( ID2 )
            {
							case Slave2_ID:
							 target_angle_RIGHTleg[0] = float_ARRY[0]; //髋
               target_angle_RIGHTleg[1] = float_ARRY[1]; //小腿
							 target_angle_RIGHTleg[2] = float_ARRY[2]; //大腿
							
               target_angle_leftleg[0]  = float_ARRY[3]; //髋
               target_angle_leftleg[1]  = float_ARRY[4]; //小腿
							 target_angle_leftleg[2]  = float_ARRY[5]; //大腿

            }
            RS485_flag = 0; //置零代表没有新数据了
//						delay_us(500);
//						printf("%.2f,%.2f,%.2f\n",target_angle_leftleg[0],target_angle_leftleg[1],target_angle_leftleg[2]);
//						printf("%d\n",RS485_dma_buffer[4]);
//						printf("%.2f,%.2f,%.2f,%.2f\n" ,float_ARRY[0],float_ARRY[1],float_ARRY[2],float_ARRY[5]);
        }
        else
        {
//            printf("CRC_ERROR\n");
//            printf("crc_cal :0x%X   crc_Frme: 0x%X",crc_cal,crc_Frme);
            RS485_flag = 0; //置零代表没有新数据了
        }
        }
        else
        {
//            printf("HEAD_ERROR\n");
            RS485_flag = 0; //置零代表没有新数据了
        }
       
    }
    }
    RS485_flag = 0;
}


//刷新ACK信号
void clear_ACK (void )
{
    Master_ACK = wait_ack;
    Slave1_ACK = wait_ack;
    Slave2_ACK = wait_ack;
    Slave3_ACK = wait_ack;
    Slave4_ACK = wait_ack;
}



//分析应答帧数据
void  Analyze_ACKfrme_data  ( uint8_t SEND_ID1  , uint8_t receive_ID2 )
{

      if ( receive_ID2 ==  MY_ID)
      {
//				printf("9\n");
        switch (SEND_ID1 )
        {
            case   MASTER_ID  :  Master_ACK = get_ack;    break;

            case   Slave1_ID  :  Slave1_ACK = get_ack;    break;

            case   Slave2_ID  :  Slave2_ACK = get_ack;     break;

            case   Slave3_ID  :  Slave3_ACK = get_ack;     break;

            case   Slave4_ID  :  Slave4_ACK = get_ack;     break;

        }
      }
}


//发送应答位回去
void  send_ACKback ( uint8_t SEND_ID1  , uint8_t receive_ID2 )  //收到了别人给我的报文
{
    uint16_t rs485_buf[3];
    rs485_buf[0] = (HEAD      << 8) | ACK_Frame;    // 包头 + 功能码
    rs485_buf[1] = (receive_ID2 << 8) | SEND_ID1;   //高位发送方低位接收方 ，我再发给别人
    rs485_buf[2] = END;
    RS485_SendBytes(rs485_buf, sizeof(rs485_buf),0);
}



//发送错误帧回去
void  send_ERRORback ( uint8_t SEND_ID1  , uint8_t receive_ID2 )  //收到了别人给我的报文
{
    uint16_t rs485_buf[3];
    rs485_buf[0] = (HEAD      << 8) | ERROR_Frame;    // 包头 + 功能码
    rs485_buf[1] = (receive_ID2 << 8) | SEND_ID1;   //高位发送方低位接收方 ，我再发给别人
    rs485_buf[2] = END;
    RS485_SendBytes(rs485_buf, sizeof(rs485_buf),0);
}





//发送数据的的次序，依次对应从机ID次序
void RS485_SET_MOTEOR_SPEED ( float *data_1 , float *data_2 , float *data_3 , float *data_4)
{

	
	MASTER_SEND_DATA(Slave1_ID, data_1,1);
	Analyze_RS485_data();
	MASTER_SEND_DATA(Slave2_ID, data_1,1);
	Analyze_RS485_data();
	MASTER_SEND_DATA(Slave3_ID, data_1,1);
	Analyze_RS485_data();
	MASTER_SEND_DATA(Slave4_ID, data_1,1);
	Analyze_RS485_data(); 
	
	
	if ( ( Slave1_ACK & Slave2_ACK & Slave3_ACK & Slave4_ACK ) == get_ack)  
	{
		clear_ACK(); //清除ACK信号
	}
	else
	{
		if(Slave1_ACK == wait_ack)  Slave1_ACK_ERR_CONT++;
		if(Slave2_ACK == wait_ack)	Slave2_ACK_ERR_CONT++;
		if(Slave3_ACK == wait_ack)	Slave3_ACK_ERR_CONT++;
		if(Slave4_ACK == wait_ack)	Slave4_ACK_ERR_CONT++;
		clear_ACK();
	}
	
	
}


