/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
#include "string.h"
#include "adc_m.h"
#include "semphr.h"
#include "RS485.h"
#include "tim.h"
#include "Servo_control.h"
#include "MYDWT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define  Calibrate_mode      0x11
#define  RS485_control_mode  0x22
#define  MODE      RS485_control_mode

int servo_start  = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
void vPrintTaskInfo(uint8_t *pucName);
/* USER CODE END Variables */
/* Definitions for RS485 */
osThreadId_t RS485Handle;
const osThreadAttr_t RS485_attributes = {
  .name = "RS485",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for SENSOR_GET */
osThreadId_t SENSOR_GETHandle;
const osThreadAttr_t SENSOR_GET_attributes = {
  .name = "SENSOR_GET",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for DUOJI_MOVE */
osThreadId_t DUOJI_MOVEHandle;
const osThreadAttr_t DUOJI_MOVE_attributes = {
  .name = "DUOJI_MOVE",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for JIAOZHUN */
osThreadId_t JIAOZHUNHandle;
const osThreadAttr_t JIAOZHUN_attributes = {
  .name = "JIAOZHUN",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for RS485RX */
osSemaphoreId_t RS485RXHandle;
const osSemaphoreAttr_t RS485RX_attributes = {
  .name = "RS485RX"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void RS485Task(void *argument);
void SENSOR_GETTask(void *argument);
void DUOJI_MOVETask(void *argument);
void JIAOZHUNTask04(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of RS485RX */
  RS485RXHandle = osSemaphoreNew(1, 0, &RS485RX_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of RS485 */
  RS485Handle = osThreadNew(RS485Task, NULL, &RS485_attributes);

  /* creation of SENSOR_GET */
  SENSOR_GETHandle = osThreadNew(SENSOR_GETTask, NULL, &SENSOR_GET_attributes);

  /* creation of DUOJI_MOVE */
  DUOJI_MOVEHandle = osThreadNew(DUOJI_MOVETask, NULL, &DUOJI_MOVE_attributes);

  /* creation of JIAOZHUN */
  JIAOZHUNHandle = osThreadNew(JIAOZHUNTask04, NULL, &JIAOZHUN_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_RS485Task */
/**
  * @brief  Function implementing the RS485 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_RS485Task */
void RS485Task(void *argument)
{
  /* USER CODE BEGIN RS485Task */
  /* Infinite loop */
  for(;;)
  {   
    #if OFF  //任务监视窗口函数
	  vPrintTaskInfo((uint8_t *)"RS485" );
	  #endif
     if(xSemaphoreTake(RS485RXHandle, portMAX_DELAY) == pdPASS )  //没有信号量我就把自己挂起来，让其他函数执行
    {	
			Analyze_RS485_data();
    }
  }
  /* USER CODE END RS485Task */
}

/* USER CODE BEGIN Header_SENSOR_GETTask */
/**
* @brief Function implementing the SENSOR_GET thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SENSOR_GETTask */
void SENSOR_GETTask(void *argument)
{
  /* USER CODE BEGIN SENSOR_GETTask */
  double  V_PRE[ADC1_CHANNEL_CNT] = { 0};        //当前电压单位是12位数值
	double  NOW_V[ADC1_CHANNEL_CNT]  ={ 0};       //当前电压单位V
  double  MY_V_PLC = 50;                      //放大倍率
	double  R_Sampling = 0.005;                  //采样电阻阻值
	double  NOW_I[ADC1_CHANNEL_CNT]  = {0};       //当前电流单位A
	double  NOW_I_FLITER[ADC1_CHANNEL_CNT]  = {0};//滤波电流单位A	
	double  I_pre[ADC1_CHANNEL_CNT]  = {0};
	int     calibrate_Count[ADC1_CHANNEL_CNT]  =     {0};
  float   I_Max = 2.4;
  /* Infinite loop */
  for(;;)
  {
    #if OFF  //任务监视窗口函数
	  vPrintTaskInfo((uint8_t *)"SENSOR_GET" );
	  #endif
    ADC_Process();
    for(int i = 0;i<ADC1_CHANNEL_CNT;i++)
    {
      V_PRE[i] = value[i] * 3.3 / 4095.0;
      NOW_V[i] = V_PRE[i] / MY_V_PLC;
      NOW_I[i] = NOW_V[i] / R_Sampling;
			NOW_I_FLITER[i] = 0.5 * NOW_I[i] + 0.5* I_pre[i];
			I_pre[i] = NOW_I[i];
      if(NOW_I_FLITER[i] >= I_Max) //校准
      {
				calibrate_Count[i] ++ ;
				
        if (calibrate_Count[i] >= 3) calibrate_flag[i] = 1;
      }
    }
//调试信息区
//		  printf("%.2f\n",Servo_zero_angle[RIGHT_LOWER_LEG_ID]);
//		  printf("%d,",MOVE_TIME[RIGHT_LOWER_LEG_ID]);
//			printf("%.2f,",CURRENT_ANGLE[RIGHT_LOWER_LEG_ID]);
//			printf("%.2f\n",NOW_I_FLITER[RIGHT_LOWER_LEG_ID]);
//		printf(",%.2f\n",CURRENT_ANGLE[RIGHT_THIGH_ID]);
//		printf(",%.2f",MOVE_TARget[RIGHT_THIGH_ID]);
//		printf(",%d\n",calibrate_flag[RIGHT_THIGH_ID]);	
//    Servo_zero_angle_Calibrate();
//    printf("\n"); 
    osDelay(5);
  }
  /* USER CODE END SENSOR_GETTask */
}

/* USER CODE BEGIN Header_DUOJI_MOVETask */
/**
* @brief Function implementing the DUOJI_MOVE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DUOJI_MOVETask */
void DUOJI_MOVETask(void *argument)
{
  /* USER CODE BEGIN DUOJI_MOVETask */

//____________模式选择_______________
  // #define   GET_CURRENT_WITHOUT_COSX     //不缓和快速电流测试
  // #define   GET_CURRENT_SLOW             //不缓和慢速电流测试
  // #define   GET_CURRENT_WITH_COSX        //缓和电流测试
  // #define   Impedance_Control            //阻抗控制测试模式
  #define   NOMALL                          //正常工作模式
  /* Infinite loop */
  for(;;)
  {
    #ifdef Impedance_Control
    Servo_Impedance_Control(Left_front_thigh_channel,90,&Left_front_thigh_angle);
    #endif  

    #if OFF  //任务监视窗口函数
	  vPrintTaskInfo((uint8_t *)"DUOJI_MOVE" );
	  #endif

    #ifdef GET_CURRENT_WITHOUT_COSX
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
    osDelay(2000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2500);
    osDelay(2000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
    osDelay(2000);
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2500);
    osDelay(2000);
    #endif


    #ifdef GET_CURRENT_SLOW

    for(int i = 0;i<2000;i++)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500+i);
      osDelay(4);
    }
  
    for(int i = 2000;i>0;i--)
    {
      __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500+i);
      osDelay(4);
    }

    #endif

    #ifdef GET_CURRENT_WITH_COSX
    if(flag == 0)
    {
      Servo_SCurve_Control(Right_thigh_channel,170,&Right_hind_thigh_angle,1000,1);
//      printf("%.2f\n",Left_front_thigh_angle);
			osDelay(1);
      if(Left_front_thigh_angle >= 169.5)
      {
        flag = 1;
      }
    }
    else if(flag == 1)
    {
      Servo_SCurve_Control(Right_thigh_channel,0,&Right_hind_thigh_angle,1000,1);
//      printf("%.2f\n",Left_front_thigh_angle);
			osDelay(1);
      if(Right_hind_thigh_angle <= 0.5)
      {
        flag = 0; 
      }
    }
    #endif
		
    #ifdef NOMALL
    //左大腿
    Servo_SCurve_Control(Left_thigh_channel,
                        MOVE_TARget[LEFT_THIGH_ID],
                        &CURRENT_ANGLE[LEFT_THIGH_ID],
                        MOVE_TIME[LEFT_THIGH_ID],
                        LEFT_THIGH_ID+1);

    //左小腿
    Servo_SCurve_Control(Left_lower_leg_channel,
                        MOVE_TARget[LEFT_LOWER_LEG_ID],
                        &CURRENT_ANGLE[LEFT_LOWER_LEG_ID],
                        MOVE_TIME[LEFT_LOWER_LEG_ID],
                        LEFT_LOWER_LEG_ID+1);

    //左髋
    Servo_SCurve_Control(Left_hip_channel,
                        MOVE_TARget[LEFT_HIP_ID],
                        &CURRENT_ANGLE[LEFT_HIP_ID],
                        MOVE_TIME[LEFT_HIP_ID],
                        LEFT_HIP_ID+1);

    //右大腿
    Servo_SCurve_Control(Right_thigh_channel,
                        MOVE_TARget[RIGHT_THIGH_ID],
                        &CURRENT_ANGLE[RIGHT_THIGH_ID],
                        MOVE_TIME[RIGHT_THIGH_ID],
                        RIGHT_THIGH_ID+1);
		// printf("MOVE_TARget%.2f\n",MOVE_TARget[RIGHT_THIGH_ID]);
		// printf("CURRENT_ANGLE%.2f\n",CURRENT_ANGLE[RIGHT_THIGH_ID]);
    //右小腿
    Servo_SCurve_Control(Right_lower_leg_channel,
                        MOVE_TARget[RIGHT_LOWER_LEG_ID],
                        &CURRENT_ANGLE[RIGHT_LOWER_LEG_ID],
                        MOVE_TIME[RIGHT_LOWER_LEG_ID],
                        RIGHT_LOWER_LEG_ID+1);


    //右髋
		Servo_SCurve_Control(Right_hip_channel,
												MOVE_TARget[RIGHT_HIP_ID],
												&CURRENT_ANGLE[RIGHT_HIP_ID],
												MOVE_TIME[RIGHT_HIP_ID],
												RIGHT_HIP_ID+1); 
    
		osDelay(1);
    #endif

  }
  /* USER CODE END DUOJI_MOVETask */
}

/* USER CODE BEGIN Header_JIAOZHUNTask04 */
/**
* @brief Function implementing the JIAOZHUN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JIAOZHUNTask04 */
void JIAOZHUNTask04(void *argument)
{
  /* USER CODE BEGIN JIAOZHUNTask04 */
  /* Infinite loop */
  for(;;)
  {   
    #if OFF  //任务监视窗口函数
	  vPrintTaskInfo((uint8_t *)"JIAOZHUN" );
	  #endif		
		
		if(MODE == Calibrate_mode)
		{
		Servo_zero_angle_Calibrate();
    osDelay(2);
		}
		else if (MODE == RS485_control_mode)
		{
			static int  CHANNEL_start = 1;
			if (CHANNEL_start)
			{
				HAL_TIM_PWM_Start(&htim1,Right_lower_leg_PWMchannel); 
				HAL_TIM_PWM_Start(&htim1,Left_lower_leg_PWMchannel); 
				CHANNEL_start = 0;
      //右腿站立
      MOVE_TARget[RIGHT_HIP_ID] =Right_hip_ZERO_angle - RIGHT_hip_SW_angle  ;
      MOVE_TARget[RIGHT_LOWER_LEG_ID] =80.51 - 30  + Right_lower_leg_ZERO_angle + 0; 
			MOVE_TARget[RIGHT_THIGH_ID]  = Right_thigh_ZERO_angle + 65.94 + 0;
      //左腿站立
      MOVE_TARget[LEFT_HIP_ID] =Left_hip_ZERO_angle  + LEFT_hip_SW_angle;
      MOVE_TARget[LEFT_LOWER_LEG_ID] = 30 - 80.51f  + Left_lower_leg_ZERO_angle - 0; 
			MOVE_TARget[LEFT_THIGH_ID]  = Left_thigh_ZERO_angle - 65.94 - 0;
			for (int i = 0;i<6 ;i ++ )
			{
					MOVE_TIME[i]  = 2000;
			}
				osDelay(2000);
			}
				
			
//				for (int i = 0;i<6 ;i ++ )
//			{
//				printf ("%d, ",pulse[i])  ;
//			}
//			printf("\n");
			
			//舵机0到180是逆时针转
			
			
			if ( target_angle_leftleg[1] != 0)
			{
				
        
			MOVE_TARget[RIGHT_HIP_ID] =Right_hip_ZERO_angle - RIGHT_hip_SW_angle + target_angle_RIGHTleg[0];
			
			//公式target_angle_RIGHTleg[1]  ==80.51 - MOVE_TARget[RIGHT_LOWER_LEG_ID] + Right_lower_leg_ZERO_angle + target_angle_RIGHTleg[2]  
			MOVE_TARget[RIGHT_LOWER_LEG_ID] =80.51 - target_angle_RIGHTleg[1]  + Right_lower_leg_ZERO_angle + target_angle_RIGHTleg[2]; 
			MOVE_TARget[RIGHT_THIGH_ID]  = Right_thigh_ZERO_angle + 65.94 + target_angle_RIGHTleg[2];
			
				 

			   
			
			MOVE_TARget[LEFT_HIP_ID] =Left_hip_ZERO_angle + target_angle_leftleg[0] + LEFT_hip_SW_angle;
				
			//公式target_angle_leftleg[1]  ==80.51 + MOVE_TARget[LEFT_LOWER_LEG_ID] - Left_lower_leg_ZERO_angle + target_angle_leftleg[2]  
			MOVE_TARget[LEFT_LOWER_LEG_ID] = target_angle_leftleg[1] - 80.51f  + Left_lower_leg_ZERO_angle - target_angle_leftleg[2]; 
			MOVE_TARget[LEFT_THIGH_ID]  = Left_thigh_ZERO_angle - 65.94 - target_angle_leftleg[2];						
					
				
			/*调试信息区
			printf("%.2f,%.2f,%.2f\n",MOVE_TARget[RIGHT_HIP_ID],MOVE_TARget[RIGHT_LOWER_LEG_ID],MOVE_TARget[RIGHT_THIGH_ID]);	
			printf("%.2f,%.2f,%.2f\n",MOVE_TARget[LEFT_HIP_ID],MOVE_TARget[LEFT_LOWER_LEG_ID],MOVE_TARget[LEFT_THIGH_ID]);
			printf("%d,%d,%d\n",pulse[RIGHT_HIP_ID],pulse[RIGHT_LOWER_LEG_ID],pulse[RIGHT_THIGH_ID]);
			printf("%.2f,%.2f,%.2f\n",target_angle_RIGHTleg[0],target_angle_RIGHTleg[1],target_angle_RIGHTleg[2]);				
			printf("%.2f,%.2f,%.2f\n",target_angle_leftleg[0],target_angle_leftleg[1],target_angle_leftleg[2]);	
			printf("R%.2f,%.2f,%.2f\n",CURRENT_ANGLE[LEFT_HIP_ID],CURRENT_ANGLE[LEFT_LOWER_LEG_ID],CURRENT_ANGLE[LEFT_THIGH_ID]);
			*/
				
				
			for (int i = 0;i<6 ;i ++ )
			{
					MOVE_TIME[i]  = 5;
			}
			osDelay(4);
		}else 
			{
      //右腿站立
      MOVE_TARget[RIGHT_HIP_ID] =Right_hip_ZERO_angle - RIGHT_hip_SW_angle  ;
      MOVE_TARget[RIGHT_LOWER_LEG_ID] =80.51 - 60  + Right_lower_leg_ZERO_angle + 30; 
			MOVE_TARget[RIGHT_THIGH_ID]  = Right_thigh_ZERO_angle + 65.94 + 30;


      //左腿站立
      MOVE_TARget[LEFT_HIP_ID] =Left_hip_ZERO_angle + target_angle_leftleg[0] + LEFT_hip_SW_angle;
      MOVE_TARget[LEFT_LOWER_LEG_ID] = 60 - 80.51f  + Left_lower_leg_ZERO_angle - 30; 
			MOVE_TARget[LEFT_THIGH_ID]  = Left_thigh_ZERO_angle - 65.94 - 30;
						for (int i = 0;i<6 ;i ++ )
			{
					MOVE_TIME[i]  = 2000;
			}
			osDelay(4);
			}
			
		}
		
  }
  /* USER CODE END JIAOZHUNTask04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
//打印函数状态信息
void vPrintTaskInfo(uint8_t *pucName)
{
    TaskHandle_t TaskHandle;
    TaskStatus_t TaskStatus;
	


    TaskHandle = xTaskGetHandle((const char *)pucName);

    vTaskGetInfo((TaskHandle_t)TaskHandle,       // 任务句柄
                 (TaskStatus_t *)&TaskStatus,     // 任务信息
                 (BaseType_t)pdTRUE,              // 允许统计任务堆栈历史最小剩余大小
                 (eTaskState)eInvalid);          // 函数自己获取任务运行状态

    printf("任务名称                : %s\r\n",TaskStatus.pcTaskName);
    printf("任务编号                : %d\r\n",(int)TaskStatus.xTaskNumber);
    // printf("任务状态                : %d\r\n",TaskStatus.eCurrentState);
    // printf("任务当前优先级          : %d\r\n",(int)TaskStatus.uxCurrentPriority);
    printf("任务基优先级            : %d\r\n",(int)TaskStatus.uxBasePriority);
    printf("任务栈基地址            : %#x\r\n",(int)TaskStatus.pxStackBase);
    printf("任务栈历史最高水位与栈顶距离 : %d\r\n",TaskStatus.usStackHighWaterMark);
    printf("\r\n");
}

/* USER CODE END Application */

