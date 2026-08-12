/**
 * @file    ws2815.h
 * @brief   WS2815 双灯带驱动 — TIM1_CH1 + TIM8_CH1, PWM+DMA 时序
 *
 * @note    协议要点 (WS2815B, 800kHz):
 *          - 每 bit 周期 = 1.25µs, 由 TIM ARR=209 @168MHz 保证 (210 计数)
 *          - '0' 高电平 ~0.35µs / '1' 高电平 ~0.70µs, 由 CCR 预编码数组控制
 *          - 每灯 24 bit, 颜色顺序 GRB
 *          - 复位 >300µs, 由帧尾 320 个低电平 bit (≈400µs) 提供
 *
 *          发送方式:
 *          - HAL_TIM_PWM_Start_DMA 把预编码好的 CCR 数组逐 bit 写入 CCR1
 *          - DMA 传输完成中断 → HAL_TIM_PWM_PulseFinishedCallback → 置空闲标志
 *          - 两条灯带各自独立 DMA 流 (DMA2_Stream5 / DMA2_Stream2), 可同时刷新
 ******************************************************************************
 */

#ifndef __WS2815_H__
#define __WS2815_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_config.h"

/* 灯带索引 */
#define WS2815_STRIP_A   0U   /**< A 条 — TIM1_CH1 (PE9) */
#define WS2815_STRIP_B   1U   /**< B 条 — TIM8_CH1 (PI5) */

/* 类型定义 ------------------------------------------------------------------*/

/** @brief 单灯 RGB 颜色 (0~255) */
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} WS2815_RGB_t;

/* 函数声明 ------------------------------------------------------------------*/

/**
 * @brief   初始化 WS2815 灯带
 * @note    使能 TIM1 的 DMA2_Stream5 中断 (DMA2_Stream2 已在 dma.c 使能),
 *          清空像素缓冲并发送一帧全灭, 使灯带进入复位状态。
 *          必须在 MX_TIM1_Init / MX_TIM8_Init 之后调用。
 */
void WS2815_Init(void);

/**
 * @brief   设置单灯颜色 (仅写入缓存, 需 WS2815_Refresh 才真正发送)
 * @param   strip   灯带索引: WS2815_STRIP_A / WS2815_STRIP_B
 * @param   index   灯珠序号 (0 ~ 灯珠数-1)
 * @param   color   RGB 颜色
 */
void WS2815_SetPixel(uint8_t strip, uint16_t index, WS2815_RGB_t color);

/**
 * @brief   整条灯带设为同一颜色 (仅写入缓存)
 * @param   strip   灯带索引
 * @param   color   RGB 颜色
 */
void WS2815_SetAll(uint8_t strip, WS2815_RGB_t color);

/**
 * @brief   刷新两条灯带 (把缓存编码成 CCR 帧并通过 DMA 发送)
 * @note    若上一帧尚未发送完则跳过本帧 (亮度平滑, 丢一帧无感)。
 *          发送为异步 DMA, 函数立即返回。
 */
void WS2815_Refresh(void);

/**
 * @brief   查询灯带是否正在发送
 * @retval  1=发送中, 0=空闲
 */
uint8_t WS2815_IsBusy(void);

/**
 * @brief   呼吸灯单步更新 — 计算下一帧亮度并写入像素缓存 + 刷新
 * @note    由 WS2815_task 周期性调用, 频率由 WS2815_BREATH_STEP_MS 决定。
 *          纯计算 + 非阻塞 DMA, 不忙等。
 */
void WS2815_BreathUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* __WS2815_H__ */
