 #ifndef JY901S_H
#define JY901S_H
 
#include <stdint.h>
#include "stm32h7xx_hal.h"
 

// JY901S 默认I2C地址（7位格式）
#define JY901S_I2C_ADDR         (0x50 << 1)  // 0xA0写 / 0xA1读
#define KEY                      0x69			//解锁寄存器地址
extern  uint16_t unlock_data ;	//解锁指令
 
 
// 寄存器地址（需根据手册确认！此处为示例）
#define JY901S_REG_RRATE        0x03    // 输出速率寄存器
#define BANDWIDTH               0x1F    // 带宽寄存器
#define BAUD                    0x04


#define JY901S_REG_ANGLE_X_L    0x3D    // X轴角度低字节
#define JY901S_REG_ANGLE_Y_L    0x3E    // Y轴角度低字节
#define JY901S_REG_ANGLE_Z_L    0x3F    // Z轴角度低字节



#define JY901S_REG_ACC_X    0x34    // X轴角度低字节
#define JY901S_REG_ACC_Y    0x35    // Y轴角度低字节
#define JY901S_REG_ACC_Z    0x36    // Z轴角度低字节
#define AXIS6								0x24    




#define RRATE_200HZ							0x0B
#define RRATE_100HZ							0x09
#define RRATE_50HZ							0x08
#define RRATE_20HZ							0x07
#define RRATE_10HZ							0x06
 




#define INTto_angle                         (180.0f / 32768.0f)


// 错误码定义
typedef enum {
    JY901S_OK = 0,
    JY901S_ERROR_I2C,
    JY901S_ERROR_TIMEOUT,
    JY901S_ERROR_ARGUMENT
} JY901S_Status;
 
// 按当前安装方向映射后的机身角度，单位：度。
typedef struct {
    float  roll;   // 左右侧倾：传感器 Y 角，左侧抬高为正
    float  pitch;  // 前后俯仰：传感器 X 角，前方抬高为正
    float  yaw;    // 航向：传感器 Z 角，逆时针增大
} JY901S_AngleData;
 



// 三轴加速度结构体（单位：米每秒方）
typedef struct {
    float ACC_X;     // X轴（Roll）
    float ACC_Y;     // Y轴（Pitch）
    float ACC_Z;     // Z轴（Yaw）
} JY901S_ACCData;


 
extern JY901S_Status my_dta;
// 函数声明
JY901S_Status JY901S_ReadRRATE(I2C_HandleTypeDef *hi2c, uint8_t *output_rate);
JY901S_Status JY901S_ReadAngles(I2C_HandleTypeDef *hi2c, JY901S_AngleData *angles);
void PrintAngles(const JY901S_AngleData *angles) ;
HAL_StatusTypeDef  MY_IIC_MEM_Transimit_UINT16_DATA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint16_t pData, uint16_t Size, uint32_t Timeout);
 
 
HAL_StatusTypeDef MY_IIC_MEM_Read_UINT8_DATA(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint16_t *pData,
    uint32_t Timeout);
 
HAL_StatusTypeDef MY_IIC_MEM_Read_UINT16_DATA(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint16_t *pData,
    uint32_t Timeout);
		
		
HAL_StatusTypeDef MY_IIC_MEM_WRITE_UINT8_DATA(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint16_t *pData,
    uint32_t Timeout);
		
//设置输出速率
HAL_StatusTypeDef JY9013S_SetDaikuan_And_Save(I2C_HandleTypeDef *hi2c) ;
HAL_StatusTypeDef STERRATE(   I2C_HandleTypeDef *hi2c, uint8_t RRATE);
void JY9013S_Unlock(I2C_HandleTypeDef *hi2c) ;
HAL_StatusTypeDef JY9013S_SetRateHz_And_Save(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef JY9013S_SetBAUD_And_Save(I2C_HandleTypeDef *hi2c) ;
void JY901SREG_init (void);
HAL_StatusTypeDef JY9013S_SetAXIS6_And_Save(I2C_HandleTypeDef *hi2c) ;
/* Monitor APIs are task-context only. Update has a single owner: JY901Task04. */
#define JY901S_READ_TIMEOUT_MS 10U
#define JY901S_SAMPLE_MS 15U
#define JY901S_PRINT_MS 100U
#define JY901S_RETRY_MS 500U
#define JY901S_STALE_MS 200U
#define JY901S_DEBUG_OUTPUT 0

typedef struct {
    JY901S_AngleData angles;
    uint32_t timestamp_ms; /* Last successful bus read; not sensor sample time. */
    uint32_t sample_count;
    uint32_t error_count;
    JY901S_Status status;
    uint8_t valid;
} JY901S_Snapshot;

JY901S_Status JY901S_Update(I2C_HandleTypeDef *hi2c);
uint8_t JY901S_GetLatest(JY901S_Snapshot *sample);
void JY901S_PrintLatest(void);
#endif // JY901S_H


