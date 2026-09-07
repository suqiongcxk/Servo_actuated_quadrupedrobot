/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include  <stdio.h>
#include "usart.h"
#include <stdint.h>
#include "time.h"
#include "MYDWT.h"
#include "task.h"
#include "RS485.h"
#include "Dog_gait.h"
#include "geometric_method.h"
#include "remote_contol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for REMOTE_Task */
osThreadId_t REMOTE_TaskHandle;
const osThreadAttr_t REMOTE_Task_attributes = {
  .name = "REMOTE_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RS485 */
osThreadId_t RS485Handle;
const osThreadAttr_t RS485_attributes = {
  .name = "RS485",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for zhuhanshu */
osThreadId_t zhuhanshuHandle;
const osThreadAttr_t zhuhanshu_attributes = {
  .name = "zhuhanshu",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for JY901 */
osThreadId_t JY901Handle;
const osThreadAttr_t JY901_attributes = {
  .name = "JY901",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RS485RTX */
osSemaphoreId_t RS485RTXHandle;
const osSemaphoreAttr_t RS485RTX_attributes = {
  .name = "RS485RTX"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void REMOTETask(void *argument);
void RS485Task02(void *argument);
void zhuhanshuTask03(void *argument);
void JY901Task04(void *argument);

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
  /* creation of RS485RTX */
  RS485RTXHandle = osSemaphoreNew(1, 0, &RS485RTX_attributes);

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
  /* creation of REMOTE_Task */
  REMOTE_TaskHandle = osThreadNew(REMOTETask, NULL, &REMOTE_Task_attributes);

  /* creation of RS485 */
  RS485Handle = osThreadNew(RS485Task02, NULL, &RS485_attributes);

  /* creation of zhuhanshu */
  zhuhanshuHandle = osThreadNew(zhuhanshuTask03, NULL, &zhuhanshu_attributes);

  /* creation of JY901 */
  JY901Handle = osThreadNew(JY901Task04, NULL, &JY901_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_REMOTETask */
/**
  * @brief  Function implementing the REMOTE_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_REMOTETask */
void REMOTETask(void *argument)
{
  /* USER CODE BEGIN REMOTETask */
  /* Infinite loop */
  for(;;)
  {

		if (Blute_tooth_flag == 1)
		{
			get_blue_tooth();
			Blute_tooth_flag = 0;

		}
   osDelay(5);
  }
  /* USER CODE END REMOTETask */
}

/* USER CODE BEGIN Header_RS485Task02 */
/**
* @brief Function implementing the RS485 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RS485Task02 */
void RS485Task02(void *argument)
{
  /* USER CODE BEGIN RS485Task02 */
  /* Infinite loop */
  for(;;)
  {
//		uint32_t ulTaskCount_RS485 = HAL_GetTick();
    //发送速度帧
    RS485_TXANGLE();
		osDelay(3);
//		vTaskDelayUntil(&ulTaskCount_RS485,5);
  }
  /* USER CODE END RS485Task02 */
}

/* USER CODE BEGIN Header_zhuhanshuTask03 */
/**
* @brief Function implementing the zhuhanshu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_zhuhanshuTask03 */
void zhuhanshuTask03(void *argument)
{
  /* USER CODE BEGIN zhuhanshuTask03 */
  /* Infinite loop */
  for(;;)
  {

    switch (GAIT_MODE)
    {
      case GAIT_MODE_SELF_ROTATION:
        Self_Rotation_move();
        break;
      case GAIT_MODE_BODY_TWIST:
        Body_Twist_move();
        break;
      case GAIT_MODE_DANCE:
        Dance_move();
        break;
      case GAIT_MODE_WAVE_LEFT_FRONT:
        Wave_Left_Front_move();
        break;
      case GAIT_MODE_FORWARD_TEST:
        Walk_Forward();
        break;
      case GAIT_MODE_TORT : 
        Tortoise_move();
        break;
      case GAIT_MODE_CRAWL : 
        Crawl_move();
        break;
      case GAIT_MODE_PACE : 
        PACE_move();
        break;
      case GAIT_MODE_BOUND : 
        Bound_move();
        break;
      case GAIT_MODE_STAND : 
        Stand_move();
        break;
      case GAIT_MODE_MARCH : 
        March_move();
        break;
			case GAIT_MODE_SIT_DOWN:
				 sit_down( );
				break;
    }
    //髋关节translate_angle_R，小腿是angle_P，大腿是theata_1
    target_angle_1[0] = LEG1_angle.translate_angle_R;
    target_angle_1[1] = LEG1_angle.angle_P;
    target_angle_1[2] = LEG1_angle.theata_1;
		
    target_angle_1[3] = LEG2_angle.translate_angle_R; 
    target_angle_1[4] = LEG2_angle.angle_P;
    target_angle_1[5] = LEG2_angle.theata_1;		
		
		
		target_angle_2[0] = LEG3_angle.translate_angle_R; 
    target_angle_2[1] = LEG3_angle.angle_P;
    target_angle_2[2] = LEG3_angle.theata_1;

    target_angle_2[3] = LEG4_angle.translate_angle_R; 
    target_angle_2[4] = LEG4_angle.angle_P;
    target_angle_2[5] = LEG4_angle.theata_1;		
		
//		printf("%.2f,%.2f\n",target_angle_1[1],target_angle_1[2]);
		osDelay(5);
  }
  /* USER CODE END zhuhanshuTask03 */
}

/* USER CODE BEGIN Header_JY901Task04 */
/**
* @brief Function implementing the JY901 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_JY901Task04 */
void JY901Task04(void *argument)
{
  /* USER CODE BEGIN JY901Task04 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END JY901Task04 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

