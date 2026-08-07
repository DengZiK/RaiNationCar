/**
 * @file    vofa.h
 * @brief   VOFA+ JustFloat 协议 — 小端浮点数组字节流
 *
 * @note    协议参考: https://www.vofa.plus/docs/learning/dataengines/justfloat/
 *
 *          数据格式 (C struct):
 *            #define CH_COUNT <N>
 *            struct Frame {
 *                float    fdata[CH_COUNT];              // 小端浮点数组
 *                uint8_t  tail[4] = {0x00,0x00,0x80,0x7f}; // 帧尾 = +Inf
 *            };
 *
 *          VOFA+ 通过帧尾 {0x00,0x00,0x80,0x7f} 在字节流中定位帧边界，
 *          不发送帧尾 → 引擎无法解析 → 缓冲区溢出 → 软件卡死。
 *
 *          STM32F4 为小端, 直接 memcpy 即可。51 单片机需调换字节序。
 ******************************************************************************
 */

#ifndef VOFA_H
#define VOFA_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/** @brief 最大通道数 (决定发送缓冲区栈占用, 8ch = 36 bytes) */
#define VOFA_MAX_CH  8U

/**
 * @brief VOFA 工作模式 — 同一串口 USART6 不能同时跑两种, 换模式需重新编译烧录
 *
 *   0 = 调参模式:  底盘任务输出浮点通道 (4轮目标/实际转速), 用 VOFA+ 绘图
 *   1 = 死机诊断模式: watchdog 任务每 100ms 输出一行文本状态帧,
 *       用串口助手 (XCOM / SSCOM / SmartRF 等) 以文本方式查看。
 *       死机后最后收到的一行 = 冻结前的系统状态, 用于区分:
 *         T 停住不变        → 中断被关 / 进了 fault 死循环 (真死机)
 *         C=0 S=L          → 遥控失联 (SBUS DMA 问题)
 *         S=M              → 电机 CAN 超时
 *         F≠0              → 具体故障码, 见 watchdog.h
 */
#define VOFA_DIAG_MODE   0

/**
 * @brief 底盘调参帧发送间隔 (Chassis 控制周期数)
 *
 *   Chassis 任务 200Hz (5ms/周期)。8 通道 JustFloat 帧在 115200 波特率下
 *   阻塞发送约 3.1ms, 若每周期都发会拖慢 200Hz 节拍、改变 dt 假设,
 *   故分频节流: 默认 10 → 每 50ms 发一帧 (20Hz)。
 *
 *   调法:
 *     波形太粗 / 想看清暂态 → 调小此值 (如 5 → 40Hz)
 *     前提是串口波特率够快 (460800+ 时可设更小)。
 *     帧太长拖慢控制 → 调大此值。
 */
#define VOFA_TUNE_DIVIDER   10U

/**
 * @brief 死机诊断状态帧 (VOFA_DIAG_MODE==1 时由 watchdog 任务每周期填充)
 * @note  motor_timeout 长度须与 app_config.h 的 MOTOR_COUNT(5) 一致
 */
typedef struct {
    uint32_t tick;              /**< HAL_GetTick(): 死机后不变 = 中断被关/进 fault */
    uint32_t fault_code;        /**< g_fault_code: 0=无故障, 1~8=见 watchdog.h */
    uint8_t  connected;         /**< rc_data.connected: 遥控是否在收 (1=在收) */
    uint8_t  sys_status;        /**< 0=OK 1=遥控失联 2=电机超时 3=急停 */
    uint8_t  stack_overflow;    /**< 1=栈溢出钩子触发过 */
    uint32_t remote_age_ms;     /**< 距上次有效遥控帧的毫秒数 */
    uint8_t  motor_timeout[5];  /**< 各电机(0~4)是否超时 */
} VOFA_Status_t;

void VOFA_Send(const float *data, uint8_t count);

/**
 * @brief 发送底盘速度环调参帧 (8 通道, JustFloat)
 * @param target_rpm[4]  4轮目标转速 (rpm, 来自 wheel_rpm_target)
 * @param actual_rpm[4]  4轮实际反馈转速 (rpm, 来自 g_motor.feedback[].speed_rpm)
 * @note  内部按 VOFA_TUNE_DIVIDER 分频节流, 阻塞发送;
 *        在 Chassis_Update 末尾调用, 勿每周期无条件发送。
 *
 *        通道排布 (VOFA+ 通道号):
 *          CH1=FR目标  CH2=FR实际  CH3=FL目标  CH4=FL实际
 *          CH5=RL目标  CH6=RL实际  CH7=RR目标  CH8=RR实际
 */
void VOFA_SendChassisTune(const float target_rpm[4], const float actual_rpm[4]);

/** @brief 发送死机诊断状态帧 (文本, 串口助手可读, 阻塞发送) */
void VOFA_SendStatus(const VOFA_Status_t *st);

#endif
