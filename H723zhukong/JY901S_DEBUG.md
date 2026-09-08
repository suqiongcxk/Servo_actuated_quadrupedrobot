# JY901S 角度串口调试

烧录 H723zhukong 工程后，已有 JY901Task04 自动运行，不需要在 main 的循环里另加调用。

## 接线和查看

- JY901S SCL → 主控 PB6（I2C1_SCL）。
- JY901S SDA → 主控 PB7（I2C1_SDA）。
- 传感器与主控共地，供电按模块规格；I2C 上拉电平须适配主控 3.3 V。当前 GPIO 配置无内部上拉，需要模块或板上已有外部上拉。
- USB 转 TTL 串口模块 RX → 主控 PD8（USART3_TX），GND → 主控 GND。仅查看输出不需要连接模块 TX。
- 串口助手：115200、8 数据位、1 停止位、无校验、无流控，文本显示。

正常示例：

    IMU t=5200 Roll=1.23 Pitch=-2.34 Yaw=15.67 deg valid=1

Roll/Pitch/Yaw 已按当前安装方向映射为机身角度，单位度：Roll 取传感器 Y 角，表示左右侧倾，左侧抬高为正；Pitch 取传感器 X 角，表示前后俯仰，前方抬高为正；Yaw 保持传感器 Z 角，逆时针增大、顺时针减小。串口输出和 GetLatest 接口均使用此映射，调用方不要再次交换 Roll/Pitch。
t 是最近成功读取的主控毫秒时间，不是传感器内部采样时间。默认地址为 7 位 0x50。
约每 20 ms 读取、每 100 ms 输出；实际更新受任务调度及传感器自身更新率影响。
未连接或读失败会输出 valid=0、status 和累计 errors，并每 500 ms 重试，恢复后自动继续输出。
status：1=I2C 错误，2=超时，3=参数错误；status=0 但 valid=0 表示数据过期。
主控原有启动延时仍然保留，等待任务启动后才会输出。

USART3 原来还发送使能/失能字节 0x01/0x02，因此操作蓝牙使能时串口助手可能显示控制字符。
现在这些发送和角度输出共用发送互斥锁，角度行不会被使能字节插入。
调试发送为有界等待，串口忙时允许丢弃一条调试输出。
printf 也接入此锁；不要在中断里调用 printf（中断调用直接返回失败）。

## 后续读取接口

包含 Core/Inc/JY901S.h，在任务中调用：

    JY901S_Snapshot imu;
    if (JY901S_GetLatest(&imu)) {
        float roll_deg = imu.angles.roll;
        float pitch_deg = imu.angles.pitch;
        float yaw_deg = imu.angles.yaw;
        /* Already mapped to the current body mounting; do not swap again. */
    }

JY901S_GetLatest 会原子复制数据。没有成功数据、最近读取失败或超过 200 ms 未更新时返回 0。
失败时保留上次角度和成功时间戳便于诊断，但不可忽略 valid 把旧角度用于控制。
sample_count 是成功总次数，error_count 是读取失败总次数。
JY901S_Update 只由 JY901Task04 调用；其他任务使用 GetLatest，不要并发操作同一 I2C 句柄。
这里只读取角度，没有自动改写传感器速率、校准、轴数配置，也没有接入机身平衡控制。

Core/Inc/JY901S.h 中 JY901S_DEBUG_OUTPUT 改为 0 可以关闭周期打印，读取和缓存仍继续。
角度为有符号 16 位、小端排列，以 180/32768 换算为度，±180 度边界的跳变应在后续姿态控制中处理。

## 验证范围

Keil 构建和主机模拟验证读取解码、超时、失败恢复、陈旧数据以及计时回绕。
用户已实测原始角度方向，本次据此交换 Roll/Pitch 赋值；交换后的输出仍需烧录确认。
