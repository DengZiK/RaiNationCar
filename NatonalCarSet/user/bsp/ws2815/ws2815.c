/**
 * @file    ws2815.c
 * @brief   WS2815 双灯带驱动实现 — PWM+DMA 逐 bit 时序
 *
 * @note    时序原理 (TIM1/TIM8, APB2 定时器时钟 = 168MHz):
 *
 *          ┌─────────────── 1 bit 周期 = 210 计数 = 1.25µs ───────────────┐
 *          │  CNT:  0 ───────── CCR ───────────── 209 (ARR) → 溢出更新    │
 *          │  输出: ██████ 高 ████  CNT<CCR 时高   ░░░ 低 ░░░░             │
 *          └──────────────────────────────────────────────────────────────┘
 *
 *          - 每溢出一次 (更新事件) 输出进入下一 bit, 而 CCR1 值在 CC1
 *            匹配时刻由 DMA 从预编码数组写入 (需 OCPreload 使能, 影子寄存器
 *            在更新事件时生效, 避免 DMA 中途改写 CCR 产生毛刺)。
 *          - '0': CCR=59  → 高 0.351µs / 低 0.899µs   (WS2815 规格内)
 *          - '1': CCR=117 → 高 0.696µs / 低 0.554µs   (WS2815 规格内)
 *          - 帧尾 320 个 CCR=0 (整周期低电平) ≈ 400µs, 满足 >300µs 复位。
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "ws2815.h"
#include "tim.h"
#include <string.h>
#include <math.h>

/* 内部宏 --------------------------------------------------------------------*/

/* 每灯 24 bit (GRB) */
#define WS2815_BITS_PER_LED           24U

/* 帧尾复位长度: 320 bit × 1.25µs = 400µs > WS2815 所需 300µs */
#define WS2815_RESET_BITS             320U

#define WS2815_STRIP_A_BITS           (WS2815_STRIP_A_LEN * WS2815_BITS_PER_LED)
#define WS2815_STRIP_B_BITS           (WS2815_STRIP_B_LEN * WS2815_BITS_PER_LED)
#define WS2815_STRIP_A_BUF_LEN        (WS2815_STRIP_A_BITS + WS2815_RESET_BITS)
#define WS2815_STRIP_B_BUF_LEN        (WS2815_STRIP_B_BITS + WS2815_RESET_BITS)

/*
 * CCR 值: 每 bit 周期 = ARR+1 = 210 计数 @168MHz
 *   '0' 高电平  59 计数 = 0.351µs
 *   '1' 高电平 117 计数 = 0.696µs
 */
#define WS2815_CCR_0                  59U
#define WS2815_CCR_1                  117U

/* 内部变量 ------------------------------------------------------------------*/

/* 两条灯带的像素缓存 (写缓存 → WS2815_Refresh 编码发送) */
static WS2815_RGB_t s_pixels_a[WS2815_STRIP_A_LEN];
static WS2815_RGB_t s_pixels_b[WS2815_STRIP_B_LEN];

/* 预编码 CCR 帧缓冲 (DMA 源, 半字) — 一次编码一帧 */
static uint16_t     s_frame_a[WS2815_STRIP_A_BUF_LEN];
static uint16_t     s_frame_b[WS2815_STRIP_B_BUF_LEN];

/* 发送忙标志 — DMA 传输完成中断里清零 */
static volatile uint8_t s_busy[2] = {0U, 0U};

/* 内部函数 ------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/*  ws2815_encode — 像素缓存 → CCR 帧 (GRB, 高位在前)                          */
/* ---------------------------------------------------------------------------*/
static void ws2815_encode(uint16_t *frame, const WS2815_RGB_t *pixels, uint16_t len)
{
    uint32_t idx = 0;

    for (uint16_t i = 0; i < len; i++) {
        /* WS2815 颜色顺序为 GRB */
        uint32_t color = ((uint32_t)pixels[i].g << 16) |
                         ((uint32_t)pixels[i].r << 8)  |
                         ((uint32_t)pixels[i].b);

        for (int32_t bit = (int32_t)WS2815_BITS_PER_LED - 1; bit >= 0; bit--) {
            frame[idx++] = ((color >> bit) & 0x01U) ? WS2815_CCR_1 : WS2815_CCR_0;
        }
    }

    /* 帧尾复位: 整周期低电平 (CCR=0) */
    for (uint16_t i = 0; i < WS2815_RESET_BITS; i++) {
        frame[idx++] = 0U;
    }
}

/* 函数实现 ------------------------------------------------------------------*/

/* ---------------------------------------------------------------------------*/
/*  WS2815_Init                                                               */
/* ---------------------------------------------------------------------------*/
void WS2815_Init(void)
{
    /*
     * 使能 CCR1 预装载 (OC1PE 位):
     * DMA 在 CC1 匹配瞬间写入 CCR1, 若不预装载会立刻改写比较值, 造成
     * '0'→'1' 位边界处输出毛刺, 灯带解码错乱。置位后 DMA 写入进入影子
     * 寄存器, 到更新事件 (bit 边界) 才生效, 每个 bit 干净完整。
     */
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM8->CCMR1 |= TIM_CCMR1_OC1PE;

    /*
     * TIM1 的 DMA (DMA2_Stream1, 随 tim.c 手动配置) 不在 dma.c 的 NVIC 列表里,
     * 这里补上它的中断使能 (DMA2_Stream2 已在 MX_DMA_Init 中使能)。
     */
    HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

    memset(s_pixels_a, 0, sizeof(s_pixels_a));
    memset(s_pixels_b, 0, sizeof(s_pixels_b));
    s_busy[0] = 0U;
    s_busy[1] = 0U;

    /* 发送一帧全灭, 让两条灯带进入复位状态 */
    WS2815_Refresh();
}

/* ---------------------------------------------------------------------------*/
/*  WS2815_SetPixel                                                           */
/* ---------------------------------------------------------------------------*/
void WS2815_SetPixel(uint8_t strip, uint16_t index, WS2815_RGB_t color)
{
    if (strip == WS2815_STRIP_A) {
        if (index < WS2815_STRIP_A_LEN) {
            s_pixels_a[index] = color;
        }
    } else {
        if (index < WS2815_STRIP_B_LEN) {
            s_pixels_b[index] = color;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  WS2815_SetAll                                                             */
/* ---------------------------------------------------------------------------*/
void WS2815_SetAll(uint8_t strip, WS2815_RGB_t color)
{
    if (strip == WS2815_STRIP_A) {
        for (uint16_t i = 0; i < WS2815_STRIP_A_LEN; i++) {
            s_pixels_a[i] = color;
        }
    } else {
        for (uint16_t i = 0; i < WS2815_STRIP_B_LEN; i++) {
            s_pixels_b[i] = color;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/*  WS2815_Refresh                                                            */
/* ---------------------------------------------------------------------------*/
void WS2815_Refresh(void)
{
    /* 任一条灯带还在发送上一帧 → 跳过本帧 (亮度平滑, 丢一帧无感) */
    if (s_busy[0] || s_busy[1]) {
        return;
    }

    ws2815_encode(s_frame_a, s_pixels_a, WS2815_STRIP_A_LEN);
    ws2815_encode(s_frame_b, s_pixels_b, WS2815_STRIP_B_LEN);

    s_busy[0] = 1U;
    s_busy[1] = 1U;

    if (HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1,
                              (uint32_t *)s_frame_a, WS2815_STRIP_A_BUF_LEN) != HAL_OK) {
        s_busy[0] = 0U;
    }
    if (HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1,
                              (uint32_t *)s_frame_b, WS2815_STRIP_B_BUF_LEN) != HAL_OK) {
        s_busy[1] = 0U;
    }
}

/* ---------------------------------------------------------------------------*/
/*  WS2815_IsBusy                                                             */
/* ---------------------------------------------------------------------------*/
uint8_t WS2815_IsBusy(void)
{
    return (s_busy[0] || s_busy[1]) ? 1U : 0U;
}

/* ---------------------------------------------------------------------------*/
/*  WS2815_BreathUpdate — 呼吸灯单步                                           */
/*                                                                            */
/*  亮度 b = (1 - cos φ) / 2 × 255, 一个周期内: 灭 → 最亮 → 灭 (正弦呼吸)      */
/*  基础色乘上亮度得到实际输出色, 两条灯带同步呼吸。                            */
/* ---------------------------------------------------------------------------*/
void WS2815_BreathUpdate(void)
{
    static const float k_two_pi = 6.28318530718f;

    /* 当前相位 0~2π (周期 WS2815_BREATH_PERIOD_MS) */
    uint32_t now_ms = HAL_GetTick() % WS2815_BREATH_PERIOD_MS;
    float phase = (float)now_ms * k_two_pi / (float)WS2815_BREATH_PERIOD_MS;

    /* 0~255 亮度: (1-cosφ)/2 → 0→255→0 */
    uint8_t b = (uint8_t)((0.5f - 0.5f * cosf(phase)) * 255.0f + 0.5f);

    WS2815_RGB_t c;
    c.r = (uint8_t)(((uint32_t)WS2815_BREATH_BASE_R * b) / 255U);
    c.g = (uint8_t)(((uint32_t)WS2815_BREATH_BASE_G * b) / 255U);
    c.b = (uint8_t)(((uint32_t)WS2815_BREATH_BASE_B * b) / 255U);

    for (uint16_t i = 0; i < WS2815_STRIP_A_LEN; i++) {
        s_pixels_a[i] = c;
    }
    for (uint16_t i = 0; i < WS2815_STRIP_B_LEN; i++) {
        s_pixels_b[i] = c;
    }

    WS2815_Refresh();
}

/* ---------------------------------------------------------------------------*/
/*  HAL_TIM_PWM_PulseFinishedCallback — PWM DMA 传输完成回调 (弱函数覆盖)       */
/*                                                                            */
/*  两条灯带的 DMA 各自完成时清对应忙标志, 释放下一帧发送机会。                 */
/* ---------------------------------------------------------------------------*/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) {
        s_busy[0] = 0U;
    } else if (htim->Instance == TIM8) {
        s_busy[1] = 0U;
    }
}
