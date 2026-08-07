/**
 * @file    imu.c
 * @brief   大疆 C 板板载 BMI088 陀螺仪驱动 + 航向积分实现
 *
 * @note    寄存器定义基于大疆官方 Development-Board-C-Examples
 *          13.spi_bmi088 驱动 (BMI088reg.h), 已按 C 板引脚校准。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "imu.h"
#include <string.h>

/* 外部 SPI1 句柄 — CubeMX 生成 (Core/Src/spi.c) */
extern SPI_HandleTypeDef hspi1;

/* ===== BMI088 陀螺仪寄存器 (CS_GYRO = PB0) ===== */
#define GYRO_CHIP_ID        0x00U    /**< 读值 0x0F */
#define GYRO_RATE_X_LSB     0x02U    /**< 数据从 0x02 起连续 6 字节 */
#define GYRO_RATE_X_MSB     0x03U
#define GYRO_RATE_Y_LSB     0x04U
#define GYRO_RATE_Y_MSB     0x05U
#define GYRO_RATE_Z_LSB     0x06U
#define GYRO_RATE_Z_MSB     0x07U
#define GYRO_RANGE          0x0FU    /**< 0x00 = ±2000dps */
#define GYRO_BANDWIDTH      0x10U    /**< 0x82 = 1000Hz ODR / 116Hz BW */
#define GYRO_LPM1           0x11U    /**< 0x00 = normal 模式 */
#define GYRO_SOFTRESET      0x14U    /**< 写 0xB6 */
#define GYRO_INT_CTRL       0x15U    /**< 0x80 = DRDY 中断 (轮询可不写) */

/* ===== BMI088 加速度计寄存器 (CS_ACC = PA4) ===== */
#define ACC_CHIP_ID         0x00U    /**< 读值 0x1E */
#define ACC_XOUT_L          0x12U    /**< 数据从 0x12 起连续 6 字节 */
#define ACC_XOUT_M          0x13U
#define ACC_YOUT_L          0x14U
#define ACC_YOUT_M          0x15U
#define ACC_ZOUT_L          0x16U
#define ACC_ZOUT_M          0x17U
#define ACC_CONF            0x40U    /**< 0xAB = normal | 800Hz | must-set(0x80) */
#define ACC_RANGE           0x41U    /**< 0x00 = ±3g */
#define ACC_PWR_CONF        0x7CU    /**< 0x00 = active */
#define ACC_PWR_CTRL        0x7DU    /**< 0x04 = 加速度计使能 */
#define ACC_SOFTRESET       0x7EU    /**< 写 0xB6 */

#define IMU_PI              3.14159265358979f
#define IMU_DEV_GYRO        1U       /**< 片选选择: 陀螺仪 */
#define IMU_DEV_ACC         0U       /**< 片选选择: 加速度计 */

/* 全局句柄定义 --------------------------------------------------------------*/
IMU_Handle_t g_imu;

/* ---------------------------------------------------------------------------*/
/*  底层 SPI 读写                                                              */
/* ---------------------------------------------------------------------------*/

static void IMU_SPI_Select(uint8_t is_gyro)
{
    if (is_gyro) {
        HAL_GPIO_WritePin(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(IMU_ACC_CS_PORT, IMU_ACC_CS_PIN, GPIO_PIN_RESET);
    }
}

static void IMU_SPI_Unselect(uint8_t is_gyro)
{
    if (is_gyro) {
        HAL_GPIO_WritePin(IMU_GYRO_CS_PORT, IMU_GYRO_CS_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(IMU_ACC_CS_PORT, IMU_ACC_CS_PIN, GPIO_PIN_SET);
    }
}

/* 写寄存器: 地址 bit7=0 */
static void IMU_WriteReg(uint8_t is_gyro, uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), data };
    uint8_t rx[2] = { 0, 0 };
    IMU_SPI_Select(is_gyro);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    IMU_SPI_Unselect(is_gyro);
}

/* 读单寄存器: 地址 bit7=1, 数据在 rx[1]
 * @note 加速度计 (is_gyro=0) SPI 读需 1 个 dummy 字节, 真实数据在 rx[2] */
static uint8_t IMU_ReadReg(uint8_t is_gyro, uint8_t reg)
{
    uint8_t tx[3] = { (uint8_t)(reg | 0x80), 0x55, 0x55 };
    uint8_t rx[3] = { 0, 0, 0 };
    uint8_t n = is_gyro ? 2U : 3U;   /* 加速度计多读 1 个 dummy */
    IMU_SPI_Select(is_gyro);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, n, 100);
    IMU_SPI_Unselect(is_gyro);
    return is_gyro ? rx[1] : rx[2];
}

/* 连读 len 字节 (寄存器自动递增), 数据在 buf[0..len-1]
 * @note 加速度计 (is_gyro=0) SPI 读需 1 个 dummy 字节, 真实数据从 rx[2] 起 */
static void IMU_ReadRegs(uint8_t is_gyro, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx[16] = { 0 };
    uint8_t rx[16] = { 0 };
    uint8_t n = len + (is_gyro ? 1U : 2U);   /* 加速度计多 1 个 dummy */
    tx[0] = (uint8_t)(reg | 0x80);
    for (uint8_t i = 1; i < n; i++) {
        tx[i] = 0x55;
    }
    IMU_SPI_Select(is_gyro);
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, n, 10);
    IMU_SPI_Unselect(is_gyro);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = rx[i + (is_gyro ? 1U : 2U)];
    }
}

/* ---------------------------------------------------------------------------*/
/*  原始数据读取                                                              */
/* ---------------------------------------------------------------------------*/

/*
 * 陀螺仪: 从 CHIP_ID(0x00) 连读 8 字节, 首字节必须为 0x0F (顺带做 SPI 帧同步校验)
 * 布局: buf[0]=CHIP_ID, buf[1]=0x01(保留), buf[2..7]=X_L..Z_H
 */
static uint8_t IMU_ReadGyroRaw(int16_t g[3])
{
    uint8_t buf[8];
    IMU_ReadRegs(IMU_DEV_GYRO, GYRO_CHIP_ID, buf, 8);
    if (buf[0] != 0x0F) {
        return 1;   /* 帧同步失败 */
    }
    g[0] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    g[1] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    g[2] = (int16_t)(((uint16_t)buf[7] << 8) | buf[6]);
    return 0;
}

/* 加速度计: 从 0x12 连读 6 字节 */
static uint8_t IMU_ReadAccelRaw(int16_t a[3])
{
    uint8_t buf[6];
    IMU_ReadRegs(IMU_DEV_ACC, ACC_XOUT_L, buf, 6);
    a[0] = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    a[1] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    a[2] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);
    return 0;
}

/* ---------------------------------------------------------------------------*/
/*  IMU_Init - BMI088 初始化                                                   */
/* ---------------------------------------------------------------------------*/
uint8_t IMU_Init(void)
{
    uint8_t id;
    uint8_t res = 0;

    memset(&g_imu, 0, sizeof(g_imu));
    g_imu.last_tick = HAL_GetTick();

    /* ---------- 陀螺仪 ---------- */
    IMU_WriteReg(IMU_DEV_GYRO, GYRO_SOFTRESET, 0xB6);
    HAL_Delay(80);
    id = IMU_ReadReg(IMU_DEV_GYRO, GYRO_CHIP_ID);
    HAL_Delay(1);
    if (id != 0x0F) {
        return 0x01;   /* 无陀螺仪 */
    }

    static const uint8_t gyro_cfg[][2] = {
        { GYRO_RANGE,     0x00 },   /* ±2000dps */
        { GYRO_BANDWIDTH, 0x82 },   /* 1000Hz ODR / 116Hz BW */
        { GYRO_LPM1,      0x00 },   /* normal 模式 (上电默认即 normal) */
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(gyro_cfg); i++) {
        IMU_WriteReg(IMU_DEV_GYRO, gyro_cfg[i][0], gyro_cfg[i][1]);
        HAL_Delay(1);
        if (IMU_ReadReg(IMU_DEV_GYRO, gyro_cfg[i][0]) != gyro_cfg[i][1]) {
            res |= 0x10;   /* 配置回读失败 */
        }
    }
    g_imu.gyro_ok = 1;

    /* ---------- 加速度计 ---------- */
    IMU_ReadReg(IMU_DEV_ACC, ACC_CHIP_ID);   /* 哑读: 强制 I2C→SPI 切换 */
    IMU_WriteReg(IMU_DEV_ACC, ACC_SOFTRESET, 0xB6);
    HAL_Delay(80);
    IMU_WriteReg(IMU_DEV_ACC, ACC_PWR_CTRL, 0x04);   /* 切 normal 模式 (suspend 读不到) */
    HAL_Delay(50);
    IMU_WriteReg(IMU_DEV_ACC, ACC_PWR_CONF, 0x00);   /* active */
    HAL_Delay(1);
    id = IMU_ReadReg(IMU_DEV_ACC, ACC_CHIP_ID);
    HAL_Delay(1);
    if (id != 0x1E) {
        return 0x02;   /* 无加速度计 */
    }

    static const uint8_t acc_cfg[][2] = {
        { ACC_CONF,     0xAB },   /* normal | 800Hz | must-set */
        { ACC_RANGE,    0x00 },   /* ±3g */
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(acc_cfg); i++) {
        IMU_WriteReg(IMU_DEV_ACC, acc_cfg[i][0], acc_cfg[i][1]);
        HAL_Delay(1);
        if (IMU_ReadReg(IMU_DEV_ACC, acc_cfg[i][0]) != acc_cfg[i][1]) {
            res |= 0x20;
        }
    }
    g_imu.accel_ok = 1;

    /* 零偏校准 (上电需整车静止) */
    IMU_CalibrateZeroBias();
    g_imu.yaw = 0.0f;
    g_imu.yaw_rate = 0.0f;
    return res;
}

/* ---------------------------------------------------------------------------*/
/*  IMU_Update - 周期更新 (200Hz)                                              */
/* ---------------------------------------------------------------------------*/
void IMU_Update(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - g_imu.last_tick) * 0.001f;

    /* 防异常: 首帧或被抢占(>20ms)时回退标称周期 5ms */
    if (dt <= 0.0f || dt > 0.02f) {
        dt = IMU_NOMINAL_DT;
    }
    g_imu.last_tick = now;

    int16_t raw[3];

    /* 陀螺仪 → 航向积分 (仅 Z 轴参与 yaw) */
    if (IMU_ReadGyroRaw(raw) == 0) {
        g_imu.gx = (float)raw[0] * IMU_GYRO_RADPS_PER_LSB;
        g_imu.gy = (float)raw[1] * IMU_GYRO_RADPS_PER_LSB;
        g_imu.gz = (float)raw[2] * IMU_GYRO_RADPS_PER_LSB;

        float wz = g_imu.gz - g_imu.gz_offset;   /* 减零偏 */
        g_imu.yaw_rate = YAW_DIR_SIGN * wz;      /* 方向符号校正 */
        g_imu.yaw += g_imu.yaw_rate * dt;

        /* 归一化到 [-pi, pi], 防 float 累积膨胀 */
        while (g_imu.yaw >  IMU_PI) g_imu.yaw -= 2.0f * IMU_PI;
        while (g_imu.yaw < -IMU_PI) g_imu.yaw += 2.0f * IMU_PI;
    }

    /* 加速度计 (备用, 不参与控制) */
    if (IMU_ReadAccelRaw(raw) == 0) {
        g_imu.ax = (float)raw[0] * IMU_ACCEL_MPS2_PER_LSB;
        g_imu.ay = (float)raw[1] * IMU_ACCEL_MPS2_PER_LSB;
        g_imu.az = (float)raw[2] * IMU_ACCEL_MPS2_PER_LSB;
    }
}

/* ---------------------------------------------------------------------------*/
/*  公共 API                                                                   */
/* ---------------------------------------------------------------------------*/

float IMU_GetYaw(void)
{
    return g_imu.yaw;
}

float IMU_GetYawRate(void)
{
    return g_imu.yaw_rate;
}

float IMU_AngleDiff(float a, float b)
{
    float diff = a - b;
    while (diff >  IMU_PI) diff -= 2.0f * IMU_PI;
    while (diff < -IMU_PI) diff += 2.0f * IMU_PI;
    return diff;
}

void IMU_CalibrateZeroBias(void)
{
    int16_t raw[3];
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    uint32_t n = 0;

    for (uint32_t i = 0; i < IMU_CALIB_SAMPLES; i++) {
        if (IMU_ReadGyroRaw(raw) == 0) {
            sx += (float)raw[0];
            sy += (float)raw[1];
            sz += (float)raw[2];
            n++;
        }
        HAL_Delay(5);   /* 200Hz 采样 */
    }

    if (n > 0) {
        g_imu.gx_offset = sx / (float)n * IMU_GYRO_RADPS_PER_LSB;
        g_imu.gy_offset = sy / (float)n * IMU_GYRO_RADPS_PER_LSB;
        g_imu.gz_offset = sz / (float)n * IMU_GYRO_RADPS_PER_LSB;
    }
}

uint8_t IMU_IsReady(void)
{
    return (g_imu.gyro_ok && g_imu.accel_ok) ? 1U : 0U;
}
