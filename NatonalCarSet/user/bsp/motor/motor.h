/**
 * @file    motor.h
 * @brief   DJI M3508 电机 CAN 通信层 (C620 电调)
 *
 * @note    通信协议:
 *          - 控制帧: STM32 → C620 电调
 *            - 底盘电机 1-4: CAN ID 0x200, 8 bytes (4×int16_t 电流值)
 *            - 升降电机 5:   CAN ID 0x1FF, 前 2 bytes (1×int16_t)
 *            电流范围: ±16384 对应 ±20A
 *
 *          - 反馈帧: C620 → STM32
 *            - 电机 1-4: CAN ID 0x201~0x204
 *            - 电机 5:   CAN ID 0x205
 *            每帧 8 bytes:
 *              [0-1] 机械角度 (0~8191, uint16_t)
 *              [2-3] 转速 rpm (int16_t)
 *              [4-5] 实际电流 (int16_t)
 *              [6]   温度 °C
 *              [7]   保留
 ******************************************************************************
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_config.h"
#include "can.h"

/*
 * 电机索引和计数宏统一在 app_config.h 中定义，motor.h 通过 include 自动获得。
 * 此处不再重复定义，避免维护两套副本导致不一致。
 */

/* 类型定义 ------------------------------------------------------------------*/

/**
 * @brief 单个电机的反馈数据 (CAN 反馈帧解析结果)
 */
typedef struct {
    uint16_t angle;             /**< 机械角度 (0 ~ 8191) */
    int16_t  speed_rpm;         /**< 实际转速 (rpm) */
    int16_t  torque_current;    /**< 实际力矩电流 */
    uint8_t  temperature;       /**< 温度 (℃) */
    uint32_t last_update_tick;  /**< 最后一次收到反馈的 tick */
} Motor_Feedback_t;

/**
 * @brief 电机控制句柄
 */
typedef struct {
    /* --- 反馈数据 (由 CAN RX 中断更新) --- */
    Motor_Feedback_t feedback[MOTOR_COUNT];

    /* --- 控制输出 (由控制任务写入, CAN TX 发送) --- */
    int16_t current_setpoint[MOTOR_COUNT];   /**< 当前指令电流值 */

    /* --- 统计 --- */
    uint32_t tx_count;          /**< CAN TX 发送次数 */
    uint32_t rx_count[MOTOR_COUNT]; /**< 各电机反馈接收次数 */
} Motor_Handle_t;

/* 全局句柄 */
extern Motor_Handle_t g_motor;

/* 函数声明 ------------------------------------------------------------------*/

/**
 * @brief   初始化电机 CAN 通信
 * @note    配置 CAN 滤波器: 接收 ID 0x201~0x208
 *          启动 CAN 接收中断
 */
void Motor_Init(void);

/**
 * @brief   CAN 接收中断回调 — 解析 M3508 反馈帧
 * @param   hcan  CAN 句柄
 * @note    在 HAL_CAN_RxFifo0MsgPendingCallback 中调用
 */
void Motor_RxCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief   发送底盘电机 1-4 的电流指令 (CAN ID 0x200)
 * @note    应在底盘控制周期结束后调用 (200Hz)
 */
void Motor_SendChassisCurrent(void);

/**
 * @brief   发送升降电机 5 的电流指令 (CAN ID 0x1FF)
 * @note    应在升降控制周期结束后调用 (100Hz)
 */
void Motor_SendLiftCurrent(void);

/**
 * @brief   检查指定电机是否通信超时
 * @param   motor_index  电机索引 (0~4)
 * @param   timeout_ms   超时阈值 (ms)
 * @retval  1=超时, 0=正常
 */
uint8_t Motor_IsTimeout(uint8_t motor_index, uint32_t timeout_ms);

/**
 * @brief   获取电机的累计圈数 (基于角度差分)
 * @param   motor_index  电机索引
 * @return  累计圈数 (float, 可含小数)
 * @note    内部维护角度历史，自动处理 0→8191 翻转
 */
float Motor_GetTotalRevolutions(uint8_t motor_index);

/**
 * @brief   重置累计圈数计数器 (用于升降机构零位校准)
 * @param   motor_index  电机索引
 */
void Motor_ResetRevolutions(uint8_t motor_index);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H__ */
