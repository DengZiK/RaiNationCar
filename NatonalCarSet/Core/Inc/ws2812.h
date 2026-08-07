/**
 * @file    ws2812.h
 * @brief   WS2812B 双灯带驱动 (TIM1 CH1 + TIM8 CH1 PWM DMA)
 *
 * 硬件映射:
 *   TIM1 CH1 (PE9, AF1) → 左灯带 DIN  — DMA2_Stream5_Ch6
 *   TIM8 CH1 (PI5, AF3) → 右灯带 DIN  — DMA2_Stream2_Ch7
 *
 * 时序基准:
 *   TIMx clock = 168MHz (APB2), Prescaler=0, Period=209 → PWM = 800kHz (1.25µs)
 *   WS_BIT0 = 67  → ~400ns high ("0" code: need 0.35µs ± 150ns)
 *   WS_BIT1 = 134 → ~800ns high ("1" code: need 0.70µs ± 150ns)
 *   Reset = 200 × 1.25µs = 250µs (>50µs requirement)
 */

#ifndef WS2812_H
#define WS2812_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 用户可配置参数
 * ========================================================================== */

#define WS_NUM_LEFT              130U    /* 左灯带 LED 数量 (TIM8 CH1) */
#define WS_NUM_RIGHT             130U    /* 右灯带 LED 数量 (TIM8 CH2) */
#define WS_NUM_MAX               130U    /* 两条灯带最大 LED 数 (取较大者) */

#define WS_BIT0                  67U     /* "0" 码高电平计数值 */
#define WS_BIT1                  134U    /* "1" 码高电平计数值 */
#define WS2812_RESET_PERIODS     200U    /* Reset 码周期数 (250µs >> 50µs) */

/* 内部计算: DMA 缓冲区大小 (取两条中较大者) */
#if WS_NUM_LEFT >= WS_NUM_RIGHT
#define WS2812_DMA_BUF_SIZE      (WS_NUM_LEFT  * 24U + WS2812_RESET_PERIODS)
#else
#define WS2812_DMA_BUF_SIZE      (WS_NUM_RIGHT * 24U + WS2812_RESET_PERIODS)
#endif

/* ==========================================================================
 * 灯带选择枚举
 * ========================================================================== */

typedef enum {
    WS_STRIP_LEFT  = 0,   /* TIM1 CH1 / PE9  (DMA2_Stream5_Ch6) */
    WS_STRIP_RIGHT = 1    /* TIM8 CH1 / PI5  (DMA2_Stream2_Ch7) */
} ws_strip_t;

/* ==========================================================================
 * 公共 API
 * ========================================================================== */

/**
 * @brief 初始化 WS2812 (清空颜色缓冲区 → 发送全黑帧)
 */
void ws2812_init(void);

/**
 * @brief 全局亮度设置
 * @param percent 0~100, 100=最亮
 */
void ws2812_set_brightness(uint8_t percent);

/**
 * @brief 获取当前全局亮度
 */
uint8_t ws2812_get_brightness(void);

/* --- 单像素 --- */

/**
 * @brief 设置指定灯带上的单颗 LED 颜色
 * @param strip 灯带选择
 * @param idx   像素索引 (0-based)
 * @param r,g,b 颜色分量 (0~255)
 */
void ws2812_set_pixel(ws_strip_t strip, uint16_t idx,
                      uint8_t r, uint8_t g, uint8_t b);

/* --- 填充颜色 --- */

/**
 * @brief 整条灯带填充纯色
 */
void ws2812_fill_color(ws_strip_t strip, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 两条灯带同时填充纯色
 */
void ws2812_fill_all(uint8_t r, uint8_t g, uint8_t b);

/* --- 发送到硬件 --- */

/**
 * @brief 将单条灯带的颜色缓冲通过 DMA 发送 (非阻塞, 内部等待上次发送完成)
 */
void ws2812_flush(ws_strip_t strip);

/**
 * @brief 同时发送两条灯带 (并行 DMA, 同时启动 CH1 和 CH2)
 */
void ws2812_flush_all(void);

/* ==========================================================================
 * 快捷颜色 API (单条灯带)
 * ========================================================================== */

void ws2812_show_red(ws_strip_t strip, uint16_t n);
void ws2812_show_green(ws_strip_t strip, uint16_t n);
void ws2812_show_blue(ws_strip_t strip, uint16_t n);
void ws2812_show_yellow(ws_strip_t strip, uint16_t n);
void ws2812_show_purple(ws_strip_t strip, uint16_t n);

/* ==========================================================================
 * LED 动画任务 (FreeRTOS)
 * ========================================================================== */

/**
 * @brief WS2812 FreeRTOS 任务入口
 *
 * 默认动画: 双灯带彩虹流水灯 (40ms 周期)
 * 可自行替换为跑马灯/呼吸/颜色切换等效果
 */
void WS2812_task(void *argument);

/* ==========================================================================
 * DMA 完成回调 (HAL 内部使用)
 * ========================================================================== */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* WS2812_H */
