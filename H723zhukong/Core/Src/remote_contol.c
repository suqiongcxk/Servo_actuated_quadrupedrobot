#include "remote_contol.h"
#include "JY901S.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "PID.h"
#include "RS485.h"
#include "math.h"
#include "stdio.h"
#include "math.h"
#include "Dog_gait.h"
#include "usart.h"




uint8_t sbus_buf[SBUS_FRAME_SIZE];
volatile uint8_t sbus_new_data = 0;
int16_t sbus_ch[6] = {0};  
char x = '\n';


#define   RE_LOW    400   //低挡位
#define   RE_mid    270		//中档位
#define   RE_hig    200  //高档位



int    Remote_V_Limit   = 0;
float  REMOTE_BASIC_L_R_V   = 0; //左右
float  REMOTE_BASIC_F_B_V   = 0; //前后
float  RE_turn_V   = 0;



int16_t sbus_ch_BAGIN[6] = {0};
int Calibrate_flag = 0;





//串口波特率跟停止位校验位要注意一下
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if ((huart == &huart7) &&
      (sbus_buf[0] == 0x0F) && 
      ((sbus_buf[24] == 0x00) || ((sbus_buf[24] & 0x0F) == 0x04))) {
    sbus_new_data = 1;
  }
  
}






					
void get_SBUS_data(void)
{
	 if (sbus_new_data) {
    sbus_new_data = 0;
    //右摇杆左到右  520-920-1320 
    sbus_ch[0]  = (sbus_buf[1]  | (sbus_buf[2]  << 8)) & 0x07FF;		
	//左摇杆前到后  1300-900-500												
    sbus_ch[1]  = ((sbus_buf[2] >> 3) | (sbus_buf[3]  << 5)) & 0x07FF; 
	//右摇杆前到后  1736-940-136
    sbus_ch[2]  = ((sbus_buf[3] >> 6) | (sbus_buf[4]  << 2) | (sbus_buf[5] << 10)) & 0x07FF;	
	//左摇杆左到右  600-1000-1400 
    sbus_ch[3]  = ((sbus_buf[5] >> 1) | (sbus_buf[6]  << 7)) & 0x07FF;    
	//左下到上 			1800-1000-200
	sbus_ch[4]  = ((sbus_buf[6] >> 4) | (sbus_buf[7]  << 4)) & 0x07FF;		
	//右下到上 			1800-1000-200 	
    sbus_ch[5]  = ((sbus_buf[7] >> 7) | (sbus_buf[8]  << 1) | (sbus_buf[9] << 9)) & 0x07FF;	
    }
			if( Calibrate_flag <10)
			{
				for ( int i = 0; i< 6; i++ )
				{
					sbus_ch_BAGIN[i]+= sbus_ch[i];
				}
				Calibrate_flag++;
			}else 
			{
				if( Calibrate_flag == 10)
				{
						for ( int i = 0; i< 6; i++ )
					{
						sbus_ch_BAGIN[i]/= 10;
					}
					Calibrate_flag=100;
				}

			}
}



/*
A5 
00 
00 
00 
00 
00 
5A 
*/

uint8_t Enable = 1;
uint8_t Disable = 2;
static uint8_t bluetooth_move_active = 0;
static uint8_t bluetooth_turn_active = 0;

static void Bluetooth_FilterMotion(void)
{
    float move = fmaxf(fabsf(REMOTE_BASIC_L_R_V), fabsf(REMOTE_BASIC_F_B_V));
    float turn = fabsf(RE_turn_V);
    /* Include exact protocol boundaries despite float conversion rounding. */
    const float stop = BT_MOTION_STOP_CM + 0.000001f;
    const float start = BT_MOTION_START_CM - 0.000001f;
    if (move <= stop) bluetooth_move_active = 0;
    else if (move >= start) bluetooth_move_active = 1;
    if (turn <= stop) bluetooth_turn_active = 0;
    else if (turn >= start) bluetooth_turn_active = 1;
    if (!bluetooth_move_active)
        REMOTE_BASIC_L_R_V = REMOTE_BASIC_F_B_V = 0.0f;
    else
    {
        /* Suppress noise on the unused axis during straight/lateral walking. */
        if (fabsf(REMOTE_BASIC_L_R_V) <= stop) REMOTE_BASIC_L_R_V = 0.0f;
        if (fabsf(REMOTE_BASIC_F_B_V) <= stop) REMOTE_BASIC_F_B_V = 0.0f;
    }
    if (!bluetooth_turn_active) RE_turn_V = 0.0f;
}

void get_blue_tooth (void )
{
        static uint8_t last_command = 0;
        uint8_t command = (uint8_t)USART6_dma_buffer[1];
        uint8_t wave_pressed = (command == 7 && last_command != 7);
        uint8_t dance_pressed = (command == 8 && last_command != 8);
        uint8_t twist_pressed = (command == 9 && last_command != 9);
        last_command = command;
        if (command >= 4 && command <= 9)
        {
            bluetooth_move_active = 0;
            bluetooth_turn_active = 0;
        }
        /* Keep test parameters fixed, but preserve driver enable/disable. */
        if (GAIT_MODE == GAIT_MODE_FORWARD_TEST)
        {
            if (USART6_dma_buffer[1] == 5)
                USART3_TransmitLocked(&Enable, 1, 20);
            else if (USART6_dma_buffer[1] == 6)
                USART3_TransmitLocked(&Disable, 1, 20);
            return;
        }
			switch (USART6_dma_buffer[1] )   //档位切换
		{
			case 1:Remote_V_Limit = RE_LOW; GAIT_MODE = GAIT_MODE_TORT ;break ; 
			case 2:Remote_V_Limit = RE_mid;GAIT_MODE = GAIT_MODE_TORT ; break ;
			case 3:Remote_V_Limit = RE_hig;GAIT_MODE = GAIT_MODE_TORT ; break ;
			case 4:GAIT_MODE = GAIT_MODE_SIT_DOWN ; break ;
			case 5:USART3_TransmitLocked(&Enable, 1, 20);break;
			case 6:USART3_TransmitLocked(&Disable, 1, 20);break;
            case 7:
                if (wave_pressed) Wave_Left_Front_Start();
                return;
            case 8:
                if (dance_pressed) Dance_Start();
                return;
            case 9:
                if (twist_pressed) Body_Twist_Start();
                return;
			default :break;
		}
		RE_turn_V = USART6_dma_buffer[2] / 32.0; //转向-3到3
		REMOTE_BASIC_L_R_V = USART6_dma_buffer[3] / 50.0;
		REMOTE_BASIC_F_B_V = USART6_dma_buffer[4] / 50.0;

        Bluetooth_FilterMotion();

        /* A neutral stick puts TORT mode into STAND.  The following Bluetooth
         * frames may contain only stick data, so restore the selected walking
         * mode when motion becomes active again.  Tortoise_move() performs the
         * existing smooth posture transition before it starts stepping. */
        if (GAIT_MODE == GAIT_MODE_STAND &&
            (bluetooth_move_active || bluetooth_turn_active) &&
            (Remote_V_Limit == RE_LOW || Remote_V_Limit == RE_mid ||
             Remote_V_Limit == RE_hig))
        {
            GAIT_MODE = GAIT_MODE_TORT;
        }

		REMOTE_V_Set(REMOTE_BASIC_L_R_V,REMOTE_BASIC_F_B_V,RE_turn_V ,Remote_V_Limit );

		/* 调试区
		printf( "%d,%.2f,%.2f,%.2f \n",Remote_V_Limit,RE_turn_V,REMOTE_BASIC_L_R_V,REMOTE_BASIC_F_B_V);
		*/
}





//遥控运动
void  REMOTE_MOVE( void  )
{
        if (GAIT_MODE == GAIT_MODE_FORWARD_TEST)
            return;
		switch (sbus_ch[5] )   //档位切换
		{
			case 1800:Remote_V_Limit = RE_LOW;GAIT_MODE = GAIT_MODE_CRAWL; break ;//最下
			case 1000:Remote_V_Limit = RE_mid;GAIT_MODE = GAIT_MODE_TORT;  break ;//中间
			case 200 :Remote_V_Limit = RE_hig;GAIT_MODE = GAIT_MODE_BOUND; break ;//最上
		}
		//右摇杆管前后左右移动
		REMOTE_BASIC_L_R_V  =  (sbus_ch[0] - sbus_ch_BAGIN[0]) / Remote_V_Limit; //-1到1
		REMOTE_BASIC_L_R_V  = __fabs(sbus_ch[0] - sbus_ch_BAGIN[0]) < 30 ? 0 : REMOTE_BASIC_L_R_V ;
		
		REMOTE_BASIC_F_B_V  =  (sbus_ch[2] - sbus_ch_BAGIN[2]) / Remote_V_Limit; //前到后 2到-2
		REMOTE_BASIC_F_B_V  = __fabs(sbus_ch[2] - sbus_ch_BAGIN[2]) < 30 ? 0 : REMOTE_BASIC_F_B_V ;
		//左摇杆管原地转向
		RE_turn_V = (sbus_ch[3] - sbus_ch_BAGIN[3] ); //-1到1
		RE_turn_V  = __fabs(sbus_ch[3] - sbus_ch_BAGIN[3]) < 30 ? 0 : RE_turn_V ;
		
		REMOTE_V_Set(REMOTE_BASIC_L_R_V,REMOTE_BASIC_F_B_V,RE_turn_V ,Remote_V_Limit );
}





//速度赋值
void  REMOTE_V_Set( float  REMOTE_BASIC_L_R_V, float  REMOTE_BASIC_F_B_V, float  RE_turn_V ,int Remote_V_Limit )
{
    /* Bluetooth TORT gears use the independent turn controller. Keep a neutral
     * or translation request for after the feet have settled on turn release.
     */
    if ((GAIT_MODE == GAIT_MODE_TORT || GAIT_MODE == GAIT_MODE_SELF_ROTATION) &&
        (RE_turn_V != 0.0f || GAIT_MODE_LAST == GAIT_MODE_SELF_ROTATION ||
         GAIT_MODE == GAIT_MODE_SELF_ROTATION))
    {
        switch (Remote_V_Limit)
        {
        case RE_LOW: Tort_height = 16.0f; foot_top_height = 7.0f; break;
        case RE_mid: Tort_height = 13.0f; foot_top_height = 6.0f; break;
        case RE_hig: Tort_height = 10.0f; foot_top_height = 5.0f; break;
        }
        forward_start = (RE_turn_V == 0.0f) ? -REMOTE_BASIC_F_B_V : 0.0f;
        forward_end = -forward_start;
        MOVE_DISTENCE_START = forward_start;
        MOVE_DISTENCE_END = forward_end;
        MOVE_DISTANCE = forward_end - forward_start;
        translate_BEGAIN = (RE_turn_V == 0.0f) ? REMOTE_BASIC_L_R_V : 0.0f;
        translate_END = -translate_BEGAIN;
        Self_Rotation_Request(RE_turn_V);
        return;
    }

	if(RE_turn_V == 0)
	{


		forward_start = -REMOTE_BASIC_F_B_V;
		forward_end =   REMOTE_BASIC_F_B_V;

		MOVE_DISTENCE_START = forward_start ;
		MOVE_DISTENCE_END   = forward_end;
		MOVE_DISTANCE = forward_end - forward_start;

        /* A centered Bluetooth command holds stance instead of marching. */
        if (GAIT_MODE == GAIT_MODE_TORT && REMOTE_BASIC_F_B_V == 0.0f &&
            REMOTE_BASIC_L_R_V == 0.0f)
        {
            switch (Remote_V_Limit)
            {
            case RE_LOW: Tort_height = 16.0f; break;
            case RE_mid: Tort_height = 13.0f; break;
            case RE_hig: Tort_height = 10.0f; break;
            }
            height_above_ground = Tort_height;
            GAIT_MODE = GAIT_MODE_STAND;
        }

		translate_BEGAIN = REMOTE_BASIC_L_R_V;
		translate_END = -REMOTE_BASIC_L_R_V;
		
    LEG1_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG1_YZ.translate_END = translate_END;
    LEG1_YZ.translate_step_length =  translate_END - translate_BEGAIN;
		
    LEG2_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG2_YZ.translate_END = translate_END;
    LEG2_YZ.translate_step_length =  translate_END - translate_BEGAIN;

    LEG3_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG3_YZ.translate_END = translate_END;
    LEG3_YZ.translate_step_length =  translate_END - translate_BEGAIN;
		
		
		
    LEG4_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG4_YZ.translate_END = translate_END;
    LEG4_YZ.translate_step_length =  translate_END - translate_BEGAIN;
		
		

		switch (Remote_V_Limit)
		{
		case RE_LOW:Tort_height = 16.0f;foot_top_height = 7;
			break;
		case RE_mid:Tort_height = 13.0f;foot_top_height = 6;
			break;
		case RE_hig:Tort_height = 10.0f;foot_top_height = 5;
			break;
			

		}
//		printf("%.2f , %.2f\n",forward_start,forward_end );


        // GAIT_Init(forward_start,forward_end, forward_height, resolution,
        // height_above_ground, L1, L2,translate_BEGAIN, translate_END, translate_HIGH,Hip_lenth);

	}else 
	{

		//原地转向,不移动
		MOVE_DISTENCE_START = 0 ;
		MOVE_DISTENCE_END   = 0;
		MOVE_DISTANCE = 0;

		//一右三左 ,二右四左
		LEG1_YZ.translate_BEGAIN = RE_turn_V;
    	LEG1_YZ.translate_END = -RE_turn_V;
		LEG1_YZ.translate_step_length = LEG1_YZ.translate_END - LEG1_YZ.translate_BEGAIN;

		LEG3_YZ.translate_BEGAIN = -RE_turn_V;
    	LEG3_YZ.translate_END = RE_turn_V;
				LEG3_YZ.translate_step_length = LEG3_YZ.translate_END - LEG3_YZ.translate_BEGAIN;

		LEG2_YZ.translate_BEGAIN = RE_turn_V;
    	LEG2_YZ.translate_END = -RE_turn_V;
				LEG2_YZ.translate_step_length = LEG2_YZ.translate_END - LEG2_YZ.translate_BEGAIN;
		
		
		LEG4_YZ.translate_BEGAIN = -RE_turn_V;
    	LEG4_YZ.translate_END = RE_turn_V;
		LEG4_YZ.translate_step_length = LEG4_YZ.translate_END - LEG4_YZ.translate_BEGAIN;

		switch (Remote_V_Limit)
		{
		case RE_LOW:Tort_height = 16.0f;foot_top_height = 7;
			break;
		case RE_mid:Tort_height = 13.0f;foot_top_height = 6;
			break;
		case RE_hig:Tort_height = 10.0f;foot_top_height = 5;
			break;
		}
//				printf("%.2f\n",Tort_height);
        // GAIT_Init(forward_start,forward_end, forward_height, resolution,
        // height_above_ground, L1, L2,translate_BEGAIN, translate_END, translate_HIGH,Hip_lenth);
	}

}
