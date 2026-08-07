/**
 * @file    ws2812.c
 * @brief   WS2812B 双灯带 PWM DMA 驱动 (TIM1 CH1 + TIM8 CH1)
 *
 * 硬件连接:
 *   PE9 (AF1 TIM1_CH1) → 左灯带 DIN  — DMA2_Stream5_Ch6
 *   PI5 (AF3 TIM8_CH1) → 右灯带 DIN  — DMA2_Stream2_Ch7
 *
 * 时序:
 *   SYSCLK = 168MHz, APB2 Timer = 168MHz
 *   TIMx Prescaler=0, Period=209 → PWM = 168M/210 = 800kHz → 1.25µs/bit
 *   "0" = 67/210 duty ≈ 32%  → 400ns high (spec: 350ns ± 150ns)
 *   "1" = 134/210 duty ≈ 64% → 800ns high (spec: 700ns ± 150ns)
 *   Reset = 200 × 1.25µs = 250µs low (spec: >50µs)
 */

#include "ws2812.h"
#include "tim.h"
#include "cmsis_os.h"

/* ==========================================================================
 * 外部引用 (tim.c 中定义, CubeMX 自动生成)
 * ========================================================================== */

extern TIM_HandleTypeDef  htim1;
extern TIM_HandleTypeDef  htim8;
extern DMA_HandleTypeDef  hdma_tim1_ch1;
extern DMA_HandleTypeDef  hdma_tim8_ch1;

/* ==========================================================================
 * 静态变量
 * ========================================================================== */

/* 颜色缓冲 (GRB order for WS2812) */
static uint8_t  ws_color_buf[2][WS_NUM_MAX * 3];

/* DMA 发送缓冲 (PWM 占空比序列) */
static uint16_t ws_dma_buf[2][WS2812_DMA_BUF_SIZE];

/* DMA 忙碌标志 (0=空闲, 1=发送中) */
static volatile uint8_t dma_busy[2];

/* 全局亮度 (0~100) */
static uint8_t ws_brightness = 100;

/* 每个 strip 的 LED 数量 */
static const uint16_t ws_num[2] = { WS_NUM_LEFT, WS_NUM_RIGHT };

/* 每个 strip 对应的定时器句柄 */
static TIM_HandleTypeDef *const htim_strip[2] = {
    &htim1,    /* WS_STRIP_LEFT  — TIM1 CH1 / PE9 */
    &htim8     /* WS_STRIP_RIGHT — TIM8 CH1 / PI5 */
};

/* 每个 strip 对应的 DMA 句柄 */
static DMA_HandleTypeDef *const hdma_strip[2] = {
    &hdma_tim1_ch1,    /* WS_STRIP_LEFT  — DMA2_Stream5_Ch6 */
    &hdma_tim8_ch1     /* WS_STRIP_RIGHT — DMA2_Stream2_Ch7 */
};

/* ==========================================================================
 * 内部辅助函数
 * ========================================================================== */

/**
 * @brief 将颜色缓冲编码为 DMA 占空比序列
 *
 * WS2812B 数据格式 (GRB):
 *   每颗 LED: G7..G0, R7..R0, B7..B0 (MSB first)
 *   每 bit: "1"=WS_BIT1 (高占空比), "0"=WS_BIT0 (低占空比)
 *   帧尾: WS2812_RESET_PERIODS 个 0 (复位信号, 线路持续低电平)
 */
static void ws2812_encode(uint8_t strip)
{
    uint32_t  idx = 0;
    uint8_t   br  = ws_brightness;
    uint16_t  num = ws_num[strip];
    uint8_t  *col = ws_color_buf[strip];
    uint16_t *dma = ws_dma_buf[strip];

    for (uint16_t led = 0; led < num; led++) {
        uint8_t g = col[led * 3 + 0];
        uint8_t r = col[led * 3 + 1];
        uint8_t b = col[led * 3 + 2];

        /* 应用全局亮度 */
        if (br != 100) {
            g = (uint16_t)g * br / 100;
            r = (uint16_t)r * br / 100;
            b = (uint16_t)b * br / 100;
        }

        /* GRB 顺序, MSB first */
        for (int8_t bit = 7; bit >= 0; bit--)
            dma[idx++] = (g & (1 << bit)) ? WS_BIT1 : WS_BIT0;
        for (int8_t bit = 7; bit >= 0; bit--)
            dma[idx++] = (r & (1 << bit)) ? WS_BIT1 : WS_BIT0;
        for (int8_t bit = 7; bit >= 0; bit--)
            dma[idx++] = (b & (1 << bit)) ? WS_BIT1 : WS_BIT0;
    }

    /* Reset 信号 (全低电平 = 0 占空比) */
    for (uint16_t i = 0; i < WS2812_RESET_PERIODS; i++)
        dma[idx++] = 0;
}

/**
 * @brief 启动单条灯带的 PWM DMA 发送
 *
 * 调用前需确保:
 *   1. dma_busy[strip] == 0 (调用者已等待)
 *   2. ws_dma_buf[strip] 已编码完成
 */
static void ws2812_start_dma(uint8_t strip)
{
    TIM_HandleTypeDef *htim = htim_strip[strip];
    DMA_HandleTypeDef *hdma = hdma_strip[strip];
    uint16_t           num  = ws_num[strip];
    uint32_t           len  = num * 24U + WS2812_RESET_PERIODS;

    /* 1. 终止之前可能残留的 DMA 传输 */
    HAL_DMA_Abort(hdma);

    /* 2. 关闭 DMA 请求 */
    __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);

    /* 3. 停止 PWM 输出 (确保空闲时线路为低电平 = WS2812 reset) */
    HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);

    /* 4. 复位计数器与标志位, CCR 清零 → 输出保持低 */
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1 | TIM_FLAG_UPDATE);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);

    /* 5. 标记忙碌 */
    dma_busy[strip] = 1;

    /* 6. 启动 PWM DMA */
    HAL_TIM_PWM_Start_DMA(htim, TIM_CHANNEL_1,
                          (uint32_t *)ws_dma_buf[strip],
                          len);
}

/* ==========================================================================
 * HAL 回调
 * ========================================================================== */

/**
 * @brief PWM DMA 脉冲完成回调
 *
 * 链路: DMA Stream IRQ → HAL_DMA_IRQHandler → XferCpltCallback
 *       → TIM_DMAPeriodElapsedCplt → 本函数
 *
 * 两条灯带使用不同定时器, 通过 htim 参数直接区分完成的是哪条。
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
        dma_busy[WS_STRIP_LEFT] = 0;
    else if (htim == &htim8)
        dma_busy[WS_STRIP_RIGHT] = 0;
}

/* ==========================================================================
 * 公共 API — 初始化 & 亮度
 * ========================================================================== */

void ws2812_init(void)
{
    /* DMA2_Stream5 NVIC (TIM1 CH1 DMA) — not managed by CubeMX IOC */
    HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    for (uint8_t s = 0; s < 2; s++) {
        for (uint16_t i = 0; i < ws_num[s]; i++) {
            ws_color_buf[s][i * 3 + 0] = 0;
            ws_color_buf[s][i * 3 + 1] = 0;
            ws_color_buf[s][i * 3 + 2] = 0;
        }
        dma_busy[s] = 0;
    }

    ws_brightness = 100;
    ws2812_flush_all();   /* 发送全黑帧 */
}

void ws2812_set_brightness(uint8_t percent)
{
    if (percent > 100) percent = 100;
    ws_brightness = percent;
}

uint8_t ws2812_get_brightness(void)
{
    return ws_brightness;
}

/* ==========================================================================
 * 公共 API — 像素操作
 * ========================================================================== */

void ws2812_set_pixel(ws_strip_t strip, uint16_t idx,
                      uint8_t r, uint8_t g, uint8_t b)
{
    if (idx >= ws_num[strip]) return;

    ws_color_buf[strip][idx * 3 + 0] = g;  /* WS2812 GRB order */
    ws_color_buf[strip][idx * 3 + 1] = r;
    ws_color_buf[strip][idx * 3 + 2] = b;
}

void ws2812_fill_color(ws_strip_t strip, uint8_t r, uint8_t g, uint8_t b)
{
    uint16_t num = ws_num[strip];
    uint8_t *col = ws_color_buf[strip];

    for (uint16_t i = 0; i < num; i++) {
        col[i * 3 + 0] = g;
        col[i * 3 + 1] = r;
        col[i * 3 + 2] = b;
    }
}

void ws2812_fill_all(uint8_t r, uint8_t g, uint8_t b)
{
    ws2812_fill_color(WS_STRIP_LEFT,  r, g, b);
    ws2812_fill_color(WS_STRIP_RIGHT, r, g, b);
}

/* ==========================================================================
 * 公共 API — DMA 发送
 * ========================================================================== */

/**
 * @brief 发送单条灯带
 *
 * 两条灯带使用独立的定时器 (TIM1 / TIM8),
 * 因此单条发送完全不影响另一条。
 */
void ws2812_flush(ws_strip_t strip)
{
    while (dma_busy[strip]) {
        osDelay(0);
    }

    ws2812_encode(strip);
    ws2812_start_dma(strip);
}

/**
 * @brief 同时发送两条灯带
 *
 * 两条灯带使用独立的定时器:
 *   编码 → 同时启动两个 PWM DMA → 各自独立完成
 */
void ws2812_flush_all(void)
{
    /* 等待两条都空闲 */
    while (dma_busy[WS_STRIP_LEFT] || dma_busy[WS_STRIP_RIGHT]) {
        osDelay(0);
    }

    /* 先编码两条 */
    ws2812_encode(WS_STRIP_LEFT);
    ws2812_encode(WS_STRIP_RIGHT);

    /* 同时启动 (两条定时器独立, 无需同步计数器) */
    ws2812_start_dma(WS_STRIP_LEFT);
    ws2812_start_dma(WS_STRIP_RIGHT);
}

/* ==========================================================================
 * 快捷颜色 API
 * ========================================================================== */

void ws2812_show_red(ws_strip_t strip, uint16_t n)
{
    uint16_t num = ws_num[strip];
    for (uint16_t i = 0; i < n && i < num; i++)
        ws2812_set_pixel(strip, i, 255, 0, 0);
    ws2812_flush(strip);
}

void ws2812_show_green(ws_strip_t strip, uint16_t n)
{
    uint16_t num = ws_num[strip];
    for (uint16_t i = 0; i < n && i < num; i++)
        ws2812_set_pixel(strip, i, 0, 255, 0);
    ws2812_flush(strip);
}

void ws2812_show_blue(ws_strip_t strip, uint16_t n)
{
    uint16_t num = ws_num[strip];
    for (uint16_t i = 0; i < n && i < num; i++)
        ws2812_set_pixel(strip, i, 0, 0, 255);
    ws2812_flush(strip);
}

void ws2812_show_yellow(ws_strip_t strip, uint16_t n)
{
    uint16_t num = ws_num[strip];
    for (uint16_t i = 0; i < n && i < num; i++)
        ws2812_set_pixel(strip, i, 60, 22, 0);
    ws2812_flush(strip);
}

void ws2812_show_purple(ws_strip_t strip, uint16_t n)
{
    uint16_t num = ws_num[strip];
    for (uint16_t i = 0; i < n && i < num; i++)
        ws2812_set_pixel(strip, i, 40, 0, 40);
    ws2812_flush(strip);
}

/* ==========================================================================
 * HSV → RGB 转换
 * ========================================================================== */

static void hsv_to_rgb(uint8_t h, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = h / 43;
    uint8_t rem    = (h - region * 43) * 6;
    uint8_t p      = 255 - rem;

    switch (region) {
        case 0: *r = 255; *g = rem; *b = 0;   break;
        case 1: *r = p;   *g = 255; *b = 0;   break;
        case 2: *r = 0;   *g = 255; *b = rem; break;
        case 3: *r = 0;   *g = p;   *b = 255; break;
        case 4: *r = rem; *g = 0;   *b = 255; break;
        default:*r = 255; *g = 0;   *b = p;   break;
    }
}

/* ==========================================================================
 * 彩虹流水灯动画 (默认)
 * ========================================================================== */

static void ws2812_rainbow_cycle(void)
{
    static uint16_t step = 0;

    uint16_t num_left  = ws_num[WS_STRIP_LEFT];
    uint16_t num_right = ws_num[WS_STRIP_RIGHT];

    for (uint16_t i = 0; i < num_left; i++) {
        uint8_t hue = (uint32_t)(i * 255 / num_left + step) & 0xFF;
        uint8_t r, g, b;
        hsv_to_rgb(hue, &r, &g, &b);
        ws2812_set_pixel(WS_STRIP_LEFT, i, r / 3, g / 3, b / 3);
    }

    for (uint16_t i = 0; i < num_right; i++) {
        uint8_t hue = (uint32_t)(i * 255 / num_right + step + 32) & 0xFF;
        uint8_t r, g, b;
        hsv_to_rgb(hue, &r, &g, &b);
        ws2812_set_pixel(WS_STRIP_RIGHT, i, r / 3, g / 3, b / 3);
    }

    step = (step + 1) & 0xFF;
    ws2812_flush_all();
}

/* ==========================================================================
 * FreeRTOS 任务
 * ========================================================================== */

void WS2812_task(void *argument)
{
    (void)argument;

    ws2812_init();
    ws2812_set_brightness(30);

    while (1) {
        ws2812_rainbow_cycle();
        osDelay(40);
    }
}
