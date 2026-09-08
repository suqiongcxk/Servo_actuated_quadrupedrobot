#include "JY901S.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <string.h>
#include  "stdio.h"
JY901S_Status my_dta ;
 extern I2C_HandleTypeDef hi2c1;
uint16_t unlock_data        =    0xB588;
/**
 * @brief 读取输出速率（RRATE）
 * @param hi2c I2C句柄
 * @param output_rate 返回的输出速率值
 * @return 状态码
 */
JY901S_Status JY901S_ReadRRATE(I2C_HandleTypeDef *hi2c, uint8_t *output_rate) {
    uint8_t reg_addr = JY901S_REG_RRATE;
    if (HAL_I2C_Mem_Read(hi2c, JY901S_I2C_ADDR, reg_addr,
                        I2C_MEMADD_SIZE_8BIT, output_rate, 1, 100) != HAL_OK) {
        return JY901S_ERROR_I2C;
    }
    return JY901S_OK;
}


/**
 * @brief 读取三轴角度（Roll/Pitch/Yaw）
 * @param hi2c I2C句柄
 * @param angles 返回的角度结构体（单位：度）
 * @return 状态码     一次0.2ms
 */
JY901S_Status JY901S_ReadAngles(I2C_HandleTypeDef *hi2c, JY901S_AngleData *angles)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status;
    if (hi2c == NULL || angles == NULL) return JY901S_ERROR_ARGUMENT;
    status = HAL_I2C_Mem_Read(hi2c, JY901S_I2C_ADDR,
                             JY901S_REG_ANGLE_X_L, I2C_MEMADD_SIZE_8BIT,
                             buf, sizeof(buf), JY901S_READ_TIMEOUT_MS);
    if (status == HAL_TIMEOUT) return JY901S_ERROR_TIMEOUT;
    if (status != HAL_OK) return JY901S_ERROR_I2C;
    /* Mounting map: sensor Y -> body roll, sensor X -> body pitch. */
    angles->roll = (int16_t)((uint16_t)buf[3] << 8 | buf[2]) * INTto_angle;
    angles->pitch = (int16_t)((uint16_t)buf[1] << 8 | buf[0]) * INTto_angle;
    angles->yaw = (int16_t)((uint16_t)buf[5] << 8 | buf[4]) * INTto_angle;
    return JY901S_OK;
}

JY901S_Status JY901S_ReadAngularRates(I2C_HandleTypeDef *hi2c, JY901S_GyroData *rates)
{
    uint8_t buf[6];
    HAL_StatusTypeDef status;
    if (hi2c == NULL || rates == NULL) return JY901S_ERROR_ARGUMENT;
    status = HAL_I2C_Mem_Read(hi2c, JY901S_I2C_ADDR,
                             JY901S_REG_GYRO_X_L, I2C_MEMADD_SIZE_8BIT,
                             buf, sizeof(buf), JY901S_READ_TIMEOUT_MS);
    if (status == HAL_TIMEOUT) return JY901S_ERROR_TIMEOUT;
    if (status != HAL_OK) return JY901S_ERROR_I2C;
    /* Same mounting map as angles: sensor Y -> body roll, X -> body pitch. */
    rates->roll_rate = (int16_t)((uint16_t)buf[3] << 8 | buf[2]) * INTto_GYRO;
    rates->pitch_rate = (int16_t)((uint16_t)buf[1] << 8 | buf[0]) * INTto_GYRO;
    rates->yaw_rate = (int16_t)((uint16_t)buf[5] << 8 | buf[4]) * INTto_GYRO;
    return JY901S_OK;
}




/**
 * @brief 读取三轴加速度（Roll/Pitch/Yaw）
 * @param hi2c I2C句柄
 * @param ACC_data 返回的加速度结构体
 * @return 状态码
 */
JY901S_Status JY901S_ReadACC(I2C_HandleTypeDef *hi2c, JY901S_ACCData *ACC_data) {
    uint8_t buf[6] = {0};
 
    // 一次性读取6个数据（X/Y/Z的高低字节）
    if (HAL_I2C_Mem_Read(hi2c, JY901S_I2C_ADDR, JY901S_REG_ACC_X  ,
                        I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK) {
        return JY901S_ERROR_I2C;
    }
 
    // 合并高低字节（小端模式）
    ACC_data->ACC_X  = (float)((buf[1] << 8) | buf[0]);  // X轴
    ACC_data->ACC_Y  = (float)((buf[3] << 8) | buf[2]);  // Y轴
    ACC_data->ACC_Z  = (float)((buf[5] << 8) | buf[4]);  // Z轴

    return JY901S_OK;
}






void PrintAngles(const JY901S_AngleData *angles) {
    // 转换为浮点数并打印（单位：度）
    printf(" %.2f, %.2f,  %.2f\n",
           angles->roll ,   // 机身左右侧倾，左侧抬高为负
           angles->pitch ,  // 机身前后俯仰，后方抬高为正、前方抬高为负
           angles->yaw );   // Z轴
    
}
 
 
 
HAL_StatusTypeDef  MY_IIC_MEM_Transimit_UINT16_DATA(I2C_HandleTypeDef *hi2c, 
                                                   uint16_t DevAddress, 
                                                   uint16_t MemAddress, 
                                                   uint16_t MemAddSize, 
                                                   uint16_t pData, 
                                                   uint16_t Size, 
                                                   uint32_t Timeout)
{
    uint8_t data_buf[2];           
    HAL_StatusTypeDef status;
 
    data_buf[0] = (pData >> 8) & 0xFF;  // 高8位
    data_buf[1] = pData & 0xFF;         // 低8位
    status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, data_buf, Size, Timeout);
    return status;
}
/**
  * @brief  从I2C设备寄存器读取16位数据（大端模式）
  * @param  hi2c: I2C句柄
  * @param  DevAddress: 设备地址（7位，左对齐）
  * @param  MemAddress: 目标寄存器地址
  * @param  MemAddSize: 寄存器地址大小（I2C_MEMADD_SIZE_8BIT或I2C_MEMADD_SIZE_16BIT）
  * @param  pData: 存储读取结果的指针
  * @param  Timeout: 超时时间（ms）
  * @retval HAL状态（HAL_OK/HAL_ERROR）
*/
 
HAL_StatusTypeDef MY_IIC_MEM_Read_UINT16_DATA(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint16_t *pData,
    uint32_t Timeout)
{
    uint8_t data_buf[2]; // 存储读取的原始字节
    HAL_StatusTypeDef status;
    // 从寄存器读取2字节数据
    status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress,
                             MemAddSize, data_buf, 2, Timeout);
    if (status != HAL_OK) {
        return status; // 读取失败直接返回
    }
    *pData = (data_buf[1] << 8) | data_buf[0];
    return HAL_OK;
}
 
 
HAL_StatusTypeDef MY_IIC_MEM_Read_UINT8_DATA(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint16_t *pData,
    uint32_t Timeout)
{
	
	uint8_t data = 0;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress,
                             MemAddSize, &data, 1, Timeout);
    if (status != HAL_OK) {
        return status; // 读取失败直接返回
    }
		
		*pData = data;
		return  status;
}
 
 
HAL_StatusTypeDef MY_IIC_MEM_WRITE_UINT8_DATA(
    I2C_HandleTypeDef *hi2c,
    uint16_t DevAddress,
    uint16_t MemAddress,
    uint16_t MemAddSize,
    uint16_t *pData,
    uint32_t Timeout)
{
	uint8_t data = *pData;
	HAL_StatusTypeDef status;
	
	
	status = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress,
                             MemAddSize, &data, 1, Timeout);
	
	
	return status;
		
}
 
HAL_StatusTypeDef STERRATE(   I2C_HandleTypeDef *hi2c, uint8_t RRATE)
{
	uint8_t data = RRATE;
	HAL_StatusTypeDef status;
	status = HAL_I2C_Mem_Write(hi2c,JY901S_I2C_ADDR,JY901S_REG_RRATE,I2C_MEMADD_SIZE_8BIT,&data,1,100);
	return status;
}
 
 
 
// 函数：向JY9013S的KEY寄存器写入解锁值0xB588（底层基于HAL_I2C库）
void JY9013S_Unlock(I2C_HandleTypeDef *hi2c) {
    uint8_t unlock_reg1[2] = {0x88, 0xB5}; // 先低位0x88，后高位0xB5
    uint8_t dev_addr = 0x50;              // 设备I2C地址
    uint8_t reg_addr = 0x69;              // KEY寄存器地址0x69
    
    // HAL库I2C写操作：设备地址+寄存器地址+数据
    HAL_I2C_Mem_Write(hi2c, 
                     dev_addr << 1,       // I2C地址左移1位（HAL库要求含读写位）
                     reg_addr, 
                     I2C_MEMADD_SIZE_8BIT, 
                     unlock_reg1, 
                     2, 
                     100);                // 超时时间100ms
}
 
 
 
HAL_StatusTypeDef JY9013S_SetRateHz_And_Save(I2C_HandleTypeDef *hi2c) {
    uint8_t dev_addr = 0x50;    // 设备IIC地址
    uint8_t rrate_addr = 0x03;  // RRATE寄存器地址
    // uint8_t save_addr = 0x00;   // 保存寄存器地址
    uint8_t rate_data[2] = {0x0B, 0x00};  // 200Hz配置值（低字节在前，高字节补0，凑16位）
    // uint8_t save_data[2] = {0x00, 0x00};  // 保存操作值（16位数据）
    
    // 1. 设置输出速率为200Hz（发送16位数据）
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, 
                                                 dev_addr << 1, 
                                                 rrate_addr, 
                                                 I2C_MEMADD_SIZE_8BIT, 
                                                 rate_data, 
                                                 2, 
                                                 100);
    if (status != HAL_OK) return status;
    
    // // 2. 保存配置到寄存器（发送16位数据）
    // status = HAL_I2C_Mem_Write(hi2c, 
    //                            dev_addr << 1, 
    //                            save_addr, 
    //                            I2C_MEMADD_SIZE_8BIT, 
    //                            save_data, 
    //                            2, 
    //                            100);
    return status;
}






//配置波特率
HAL_StatusTypeDef JY9013S_SetBAUD_And_Save(I2C_HandleTypeDef *hi2c) {
    uint8_t dev_addr = 0x50;    // 设备IIC地址
    uint8_t rrate_addr = BAUD;  // RRATE寄存器地址
    uint8_t rate_data[2] = {0x07, 0x00};  // 230400配置值（低字节在前，高字节补0，凑16位）
    
    // 1. 设置输出速率为256Hz（发送16位数据）
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, 
                                                 dev_addr << 1, 
                                                 rrate_addr, 
                                                 I2C_MEMADD_SIZE_8BIT, 
                                                 rate_data, 
                                                 2, 
                                                 100);
    if (status != HAL_OK) return status;
    
    return status;
}






//高刷新率采用六轴算法
HAL_StatusTypeDef JY9013S_SetAXIS6_And_Save(I2C_HandleTypeDef *hi2c) {
    uint8_t dev_addr = 0x50;    // 设备IIC地址
    uint8_t rrate_addr = AXIS6;  // RRATE寄存器地址
    uint8_t rate_data[2] = {0x01, 0x00};  // 六轴配置值（低字节在前，高字节补0，凑16位）
    
    // 1. 设置输出速率为256Hz（发送16位数据）
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, 
                                                 dev_addr << 1, 
                                                 rrate_addr, 
                                                 I2C_MEMADD_SIZE_8BIT, 
                                                 rate_data, 
                                                 2, 
                                                 100);
    if (status != HAL_OK) return status;
    
    return status;
}




//配置带宽
HAL_StatusTypeDef JY9013S_SetDaikuan_And_Save(I2C_HandleTypeDef *hi2c) {
    uint8_t dev_addr = 0x50;    // 设备IIC地址
    uint8_t rrate_addr = 0x1F;  // RRATE寄存器地址
    uint8_t save_addr = 0x00;   // 保存寄存器地址
    uint8_t rate_data[2] = {0x00, 0x00};  // 256Hz配置值（低字节在前，高字节补0，凑16位）
    uint8_t save_data[2] = {0x00, 0x00};  // 保存操作值（16位数据）
    
    // 1. 设置输出速率为256Hz（发送16位数据）
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c, 
                                                 dev_addr << 1, 
                                                 rrate_addr, 
                                                 I2C_MEMADD_SIZE_8BIT, 
                                                 rate_data, 
                                                 2, 
                                                 100);
    if (status != HAL_OK) return status;
    
    // 2. 保存配置到寄存器（发送16位数据）
    status = HAL_I2C_Mem_Write(hi2c, 
                               dev_addr << 1, 
                               save_addr, 
                               I2C_MEMADD_SIZE_8BIT, 
                               save_data, 
                               2, 
                               100);
    return status;
}






void JY901SREG_init (void)
{
	uint16_t lock_status = 0x0000;
	JY9013S_Unlock(&hi2c1);
	HAL_StatusTypeDef my_status;
	// 3. 验证是否解锁成功（读取0x69）
    
	MY_IIC_MEM_Read_UINT16_DATA(&hi2c1, JY901S_I2C_ADDR, KEY, I2C_MEMADD_SIZE_8BIT, &lock_status, 100);
	printf("Lock Status: 0x%04X\n", lock_status); // 预期输出0xB588
	// 2. 立即配置输出速率（200Hz ）带宽256Hz
	JY9013S_SetRateHz_And_Save(&hi2c1);
	JY9013S_SetBAUD_And_Save(&hi2c1);
    JY9013S_SetAXIS6_And_Save(&hi2c1);
  my_status =  JY9013S_SetDaikuan_And_Save(&hi2c1);
  if(my_status == HAL_OK )printf("save_ok\n");
	// 读取当前输出速率
	uint8_t current_rate;
	HAL_I2C_Mem_Read(&hi2c1, JY901S_I2C_ADDR, JY901S_REG_RRATE, I2C_MEMADD_SIZE_8BIT, &current_rate, 1, 100);
	printf("Current Rate: 0x%02X\n", current_rate); // 预期输出0x0B（200Hz）
	HAL_I2C_Mem_Read(&hi2c1,  JY901S_I2C_ADDR ,BANDWIDTH,  I2C_MEMADD_SIZE_8BIT, &current_rate, 1, 100);
	printf("Current BANDWIDTH: 0x%02X\n", current_rate); // 预期输出0x00（256Hz）
	HAL_I2C_Mem_Read(&hi2c1,  JY901S_I2C_ADDR ,BAUD,  I2C_MEMADD_SIZE_8BIT, &current_rate, 1, 100);
	printf("Current BAUD: 0x%02X\n", current_rate); 
	HAL_I2C_Mem_Read(&hi2c1,  JY901S_I2C_ADDR ,AXIS6,  I2C_MEMADD_SIZE_8BIT, &current_rate, 1, 100);
	printf("Current AXIS6: 0x%02X\n", current_rate); //0x00
  
}



static JY901S_Snapshot latest_sample = {
    {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f},
    0U, 0U, 0U, JY901S_ERROR_I2C, 0U
};

JY901S_Status JY901S_Update(I2C_HandleTypeDef *hi2c)
{
    JY901S_AngleData angles;
    JY901S_GyroData rates;
    JY901S_Status status = JY901S_ReadAngles(hi2c, &angles);
    if (status == JY901S_OK)
        status = JY901S_ReadAngularRates(hi2c, &rates);
    uint32_t now = HAL_GetTick();
    taskENTER_CRITICAL();
    latest_sample.status = status;
    latest_sample.valid = (status == JY901S_OK);
    if (status == JY901S_OK) {
        latest_sample.angles = angles;
        latest_sample.rates = rates;
        latest_sample.timestamp_ms = now;
        latest_sample.sample_count++;
    } else {
        latest_sample.error_count++;
    }
    taskEXIT_CRITICAL();
    return status;
}

uint8_t JY901S_GetLatest(JY901S_Snapshot *sample)
{
    if (sample == NULL) return 0U;
    taskENTER_CRITICAL();
    *sample = latest_sample;
    taskEXIT_CRITICAL();
    if ((uint32_t)(HAL_GetTick() - sample->timestamp_ms) > JY901S_STALE_MS)
        sample->valid = 0U;
    return sample->valid;
}

void JY901S_PrintLatest(void)
{
    JY901S_Snapshot sample;
    char line[160];
    int length;
    JY901S_GetLatest(&sample);
    if (sample.valid) {
        length = snprintf(line, sizeof(line),
            "IMU t=%lu Roll=%.2f Pitch=%.2f Yaw=%.2f deg Rr=%.1f Pr=%.1f dps valid=1\r\n",
            (unsigned long)sample.timestamp_ms,
            (double)sample.angles.roll, (double)sample.angles.pitch,
            (double)sample.angles.yaw, (double)sample.rates.roll_rate,
            (double)sample.rates.pitch_rate);
    } else {
        length = snprintf(line, sizeof(line),
            "IMU t=%lu valid=0 status=%u errors=%lu (check I2C1 PB6/PB7, addr=0x50)\r\n",
            (unsigned long)HAL_GetTick(), (unsigned)sample.status,
            (unsigned long)sample.error_count);
    }
    if (length > 0 && length < (int)sizeof(line))
        USART3_TransmitLocked((const uint8_t *)line, (uint16_t)length, 20U);
}
