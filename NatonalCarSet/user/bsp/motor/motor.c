/**
 * @file    motor.c
 * @brief   M3508 电机 CAN 通信层实现
 *
 * @note    CAN1 滤波器配置:
 *          - Filter Bank 0: 接收 ID 0x201~0x208 (M3508 反馈帧)
 *          - 使用标识符掩码模式
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"
#include <string.h>

/* 外部变量引用 --------------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;

/* 全局句柄定义 --------------------------------------------------------------*/
Motor_Handle_t g_motor;

/* 内部变量 — 圈数累积用 ----------------------------------------------------*/
static uint16_t g_last_angle[MOTOR_COUNT];       /**< 各电机上一次角度值 */
static int32_t  g_round_count[MOTOR_COUNT];      /**< 各电机圈数计数器 (正转+1, 反转-1) */

/* 函数实现 ------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/*  Motor_Init                                                                */
/* ---------------------------------------------------------------------------*/
void Motor_Init(void)
{
    memset(&g_motor, 0, sizeof(g_motor));

    /* 初始化圈数追踪 */
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        g_last_angle[i]  = 0;
        g_round_count[i] = 0;
        g_motor.current_setpoint[i] = 0;
    }

    /*
     * 配置 CAN 滤波器:
     * 接收 M3508 反馈帧 ID 0x201 ~ 0x208 (共 8 个 ID)
     *
     * Filter 参数:
     *   FilterId   = 0x201 << 5 (标准帧 ID 左移 5 位到 STID[10:0] 位置)
     *   FilterMask = 0x7F8 << 5 (只比较 bit[10:3], 忽略 bit[2:0])
     *               即接受 0x201~0x208 范围内的所有 ID
     */
    CAN_FilterTypeDef can_filter;
    can_filter.FilterBank           = 0;                     /* 使用 Filter Bank 0 */
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK; /* 掩码模式 */
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT; /* 32-bit 滤波器 */
    can_filter.FilterIdHigh         = (0x201U << 5);         /* 期望 ID 0x201 */
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = (0x7F8U << 5);         /* 掩码: bit[10:3] 匹配 */
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;          /* 使用 FIFO0 */
    can_filter.FilterActivation     = ENABLE;
    can_filter.SlaveStartFilterBank = 0;

    if (HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK) {
        Error_Handler();
    }

    /* 启动 CAN, 使能 FIFO0 消息挂起中断 */
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
}

/* ---------------------------------------------------------------------------*/
/*  Motor_RxCallback - CAN FIFO0 消息挂起回调                                   */
/*                                                                              */
/*  @note   此函数在 HAL_CAN_RxFifo0MsgPendingCallback 中调用                    */
/*          解析 M3508 反馈帧, 更新 g_motor.feedback[]                           */
/* ---------------------------------------------------------------------------*/
void Motor_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    /* 根据 CAN ID 确定电机索引: 0x201→0, 0x202→1, 0x203→2, 0x204→3, 0x205→4 */
    uint8_t motor_idx;
    if (rx_header.StdId >= CAN_RX_ID_BASE && rx_header.StdId < CAN_RX_ID_BASE + MOTOR_COUNT) {
        motor_idx = (uint8_t)(rx_header.StdId - CAN_RX_ID_BASE);
    } else {
        return;  /* 非预期的 CAN ID */
    }

    /* 解析反馈帧 (大端序 / Motorola format) */
    Motor_Feedback_t *fb = &g_motor.feedback[motor_idx];

    fb->angle          = ((uint16_t)rx_data[0] << 8) | (uint16_t)rx_data[1];
    fb->speed_rpm      =  (int16_t)(((uint16_t)rx_data[2] << 8) | (uint16_t)rx_data[3]);
    fb->torque_current =  (int16_t)(((uint16_t)rx_data[4] << 8) | (uint16_t)rx_data[5]);
    fb->temperature    = rx_data[6];
    fb->last_update_tick = HAL_GetTick();

    g_motor.rx_count[motor_idx]++;

    /*
     * 圈数累积 (处理 0↔8191 翻转)
     *
     * 每个电机独立初始化: 第一次收到反馈时只记录基准角度, 不计算圈数。
     * rx_count == 1 表示首次, 跳过本轮, 下次再开始累积。
     */
    if (g_motor.rx_count[motor_idx] == 1) {
        g_last_angle[motor_idx] = fb->angle;
        return;
    }

    int16_t delta = (int16_t)(fb->angle - g_last_angle[motor_idx]);

    /* 判断是否发生了翻转 (角度跳变 > 半圈即视为翻转) */
    if (delta > 4096) {
        /* 正向翻转 → 上一圈: 角度从接近 8191 跳回 0 */
        g_round_count[motor_idx]--;
    } else if (delta < -4096) {
        /* 反向翻转 → 上一圈: 角度从 0 跳变到接近 8191 */
        g_round_count[motor_idx]++;
    }

    g_last_angle[motor_idx] = fb->angle;
}

/* ---------------------------------------------------------------------------*/
/*  Motor_SendChassisCurrent — 发送底盘 4 电机电流 (CAN ID 0x200)              */
/* ---------------------------------------------------------------------------*/
void Motor_SendChassisCurrent(void)
{
    /*
     * 总线状态保护: 仅 RESET / ERROR / SLEEP 时跳过发送
     * READY / BUSY / LISTENING 都允许发送
     */
    HAL_CAN_StateTypeDef can_state = HAL_CAN_GetState(&hcan1);
    if (can_state == HAL_CAN_STATE_RESET || can_state == HAL_CAN_STATE_ERROR) {
        return;
    }

    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = CAN_TX_ID_CHASSIS;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    /* 打包 4 路 int16_t 电流值 (大端序) */
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        int16_t current = g_motor.current_setpoint[i];
        tx_data[i * 2]     = (uint8_t)((current >> 8) & 0xFF);
        tx_data[i * 2 + 1] = (uint8_t)(current & 0xFF);
    }

    /* 发送 (非阻塞: 使用邮箱, 若全忙则丢弃本帧) */
    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        /* CAN 发送失败 (如邮箱满), 静默丢弃 */
    } else {
        g_motor.tx_count++;
    }
}

/* ---------------------------------------------------------------------------*/
/*  Motor_SendLiftCurrent — 发送升降电机电流 (CAN ID 0x1FF)                     */
/* ---------------------------------------------------------------------------*/
void Motor_SendLiftCurrent(void)
{
    HAL_CAN_StateTypeDef can_state = HAL_CAN_GetState(&hcan1);
    if (can_state == HAL_CAN_STATE_RESET || can_state == HAL_CAN_STATE_ERROR) {
        return;
    }
    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.StdId = CAN_TX_ID_LIFT;
    tx_header.ExtId = 0;
    tx_header.IDE   = CAN_ID_STD;
    tx_header.RTR   = CAN_RTR_DATA;
    tx_header.DLC   = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    int16_t current = g_motor.current_setpoint[MOTOR_LIFT];

    /* 仅前 2 字节有效 (电机 5), 其余填 0 */
    tx_data[0] = (uint8_t)((current >> 8) & 0xFF);
    tx_data[1] = (uint8_t)(current & 0xFF);
    tx_data[2] = 0x00;
    tx_data[3] = 0x00;
    tx_data[4] = 0x00;
    tx_data[5] = 0x00;
    tx_data[6] = 0x00;
    tx_data[7] = 0x00;

    uint32_t tx_mailbox;
    HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &tx_mailbox);
}

/* ---------------------------------------------------------------------------*/
/*  Motor_IsTimeout                                                             */
/* ---------------------------------------------------------------------------*/
uint8_t Motor_IsTimeout(uint8_t motor_index, uint32_t timeout_ms)
{
    if (motor_index >= MOTOR_COUNT) return 1;

    uint32_t now   = HAL_GetTick();
    uint32_t last  = g_motor.feedback[motor_index].last_update_tick;

    /* 首次尚未收到反馈视为正常 (不计超时, 由调用方判断) */
    if (last == 0) return 0;

    return ((now - last) > timeout_ms) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------*/
/*  Motor_GetTotalRevolutions                                                  */
/* ---------------------------------------------------------------------------*/
float Motor_GetTotalRevolutions(uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return 0.0f;

    uint16_t angle = g_motor.feedback[motor_index].angle;
    int32_t  rounds = g_round_count[motor_index];

    /* 总圈数 = 圈数计数 + 当前角度 / 8192 */
    float total = (float)rounds + (float)angle / (float)LIFT_ENCODER_RESOLUTION;

    return total;
}

/* ---------------------------------------------------------------------------*/
/*  Motor_ResetRevolutions                                                     */
/* ---------------------------------------------------------------------------*/
void Motor_ResetRevolutions(uint8_t motor_index)
{
    if (motor_index >= MOTOR_COUNT) return;

    g_round_count[motor_index] = 0;
    g_last_angle[motor_index]  = g_motor.feedback[motor_index].angle;
}
