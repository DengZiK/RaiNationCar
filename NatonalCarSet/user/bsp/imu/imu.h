/**
 * @file    imu.h
 * @brief   大疆 C 板板载 BMI088 陀螺仪驱动 + 航向积分 (SPI1)
 *
 * @note    硬件连接 (板载, 无需外部接线):
 *          - SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (AF5)
 *          - CS_GYRO = PB0, CS_ACC = PA4 (低电平选中, 默认高)
 *          - SPI Mode 3, 8bit, 预分频16 → 5.25MHz
 *
 *          CubeMX 需使能 SPI1 并生成 hspi1 (Core/Src/spi.c)
 ******************************************************************************
 */

#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_config.h"
#include "spi.h"

/* 类型定义 ------------------------------------------------------------------*/

/**
 * @brief IMU 数据句柄
 */
typedef struct {
    uint8_t  gyro_ok;       /**< 陀螺仪初始化成功标志 */
    uint8_t  accel_ok;      /**< 加速度计初始化成功标志 */

    float    gx, gy, gz;    /**< 陀螺仪角速度 (rad/s, 原始值, 未减零偏) */
    float    ax, ay, az;    /**< 加速度 (m/s^2, 备用, 不参与控制) */

    float    gx_offset;     /**< 陀螺仪 X 零偏 (rad/s) */
    float    gy_offset;     /**< 陀螺仪 Y 零偏 (rad/s) */
    float    gz_offset;     /**< 陀螺仪 Z 零偏 (rad/s) */

    float    yaw;           /**< 航向角 (rad, [-pi, pi]) */
    float    yaw_rate;      /**< 航向角速度 (rad/s, 已做方向符号校正) */

    uint32_t last_tick;     /**< 上次积分时刻 (HAL tick) */
} IMU_Handle_t;

/* 全局句柄 */
extern IMU_Handle_t g_imu;

/* 函数声明 ------------------------------------------------------------------*/

/**
 * @brief   初始化 SPI1 + BMI088, 并做零偏校准
 * @retval  0=成功; 0x01=无陀螺仪; 0x02=无加速度计; 0x10/0x20=配置回读失败
 * @note    含阻塞校准 (~1s), 调用期间整车必须静止
 */
uint8_t IMU_Init(void);

/**
 * @brief   周期更新 (200Hz) — 读陀螺仪/加速度计, 积分航向
 * @note    由 Chassis_Update 以 200Hz 调用
 */
void IMU_Update(void);

/**
 * @brief   获取当前航向 (rad, [-pi, pi])
 */
float IMU_GetYaw(void);

/**
 * @brief   获取当前航向角速度 (rad/s)
 */
float IMU_GetYawRate(void);

/**
 * @brief   两角最短角差 = a - b, 归一化到 [-pi, pi]
 * @note    用于 PID 误差计算, 跨 ±180° 走最短路径
 */
float IMU_AngleDiff(float a, float b);

/**
 * @brief   重新做零偏校准 (需整车静止)
 */
void IMU_CalibrateZeroBias(void);

/**
 * @brief   陀螺仪与加速度计均初始化成功?
 */
uint8_t IMU_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
