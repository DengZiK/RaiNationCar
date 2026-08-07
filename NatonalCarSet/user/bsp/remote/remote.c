#include "remote.h"
#include "usart.h"
#include "string.h"

/* 外部声明 — USART3 */
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;

RC_Data_t rc_data;
uint8_t SBUS_RxBuffer[SBUS_FRAME_SIZE] = {0};
uint8_t sbus_data[SBUS_FRAME_SIZE] = {0};
static volatile uint32_t last_valid_frame_tick = 0;

static void RC_Failsafe(void);

/* ---------------------------------------------------------------------------*/
/*  遥控器初始化                                                               */
/* ---------------------------------------------------------------------------*/
HAL_StatusTypeDef remote_control_init(void)
{
    RC_Failsafe();
    last_valid_frame_tick = HAL_GetTick();
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    return HAL_UART_Receive_DMA(&huart3, SBUS_RxBuffer, SBUS_FRAME_SIZE);
}

/* ---------------------------------------------------------------------------*/
/*  遥控器超时检测 (需周期调用)                                                 */
/* ---------------------------------------------------------------------------*/
void remote_control_watchdog(void)
{
    if (rc_data.connected &&
        (uint32_t)(HAL_GetTick() - last_valid_frame_tick) > SBUS_TIMEOUT_MS)
    {
        RC_Failsafe();
    }
}

/* 距上次有效遥控帧的毫秒数 — 供死机诊断状态帧使用
 * 持续增大 → SBUS 已停 (DMA 死/信号断) */
uint32_t remote_control_frame_age(void)
{
    return (uint32_t)(HAL_GetTick() - last_valid_frame_tick);
}

/* ---------------------------------------------------------------------------*/
/*  UART 错误回调 — SBUS DMA 自动复活 (关键修复)                                */
/* ---------------------------------------------------------------------------*/
/*
 * 死机根因 (HAL 源码 stm32f4xx_hal_uart.c 已核实):
 *   默认 HAL 在 UART 接收出错 (奇偶错 PE / 帧错 FE / 噪声 NE / 溢出 ORE)
 *   且处于 DMA 接收模式时, 会:
 *     1. UART_EndRxTransfer()   → 关闭接收
 *     2. HAL_DMA_Abort_IT()     → 永久停掉 DMA 循环接收
 *     3. 调用 __weak 空实现 HAL_UART_ErrorCallback
 *   由于本项目未定义该回调, DMA 一旦被停就再也不会重启 →
 *   SBUS 一字节出错 → 遥控器永久失联 → 车"死机", 只能重新上电。
 *
 * 修复: 在此回调中清错误标志 + 重新启动 DMA 循环接收, 让 SBUS 自动恢复。
 *       此回调在 HAL 的 DMA Abort 完成后调用 (DMA 已完全停止,
 *       huart->RxState 已回到 READY), 此时可安全地重新开始接收。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        /* 清 UART 错误标志: 读 SR 再读 DR, 一次性清 ORE/FE/NE/PE。
         * 不清的话, 错误标志会残留, 导致下次错误中断误触发。 */
        __HAL_UART_CLEAR_OREFLAG(huart);

        /* 重新启动 DMA 循环接收 (SBUS_RxBuffer / SBUS_FRAME_SIZE 见 remote.h) */
        (void)HAL_UART_Receive_DMA(huart, SBUS_RxBuffer, SBUS_FRAME_SIZE);
    }
}

/* ---------------------------------------------------------------------------*/
/*  通用通道数据转换 (在映射后的 200~1800 值域上做死区，保证死区一致性)            */
/* ---------------------------------------------------------------------------*/
static int16_t sbus_to_rc(uint16_t sbus_val)
{
    /* 先线性映射: 0~2047  ->  200~1800 */
    int32_t mapped = (int32_t)(sbus_val * 1600 / 2047) + 200;
    /* 限幅到 [200, 1800] */
    if (mapped > 1800) mapped = 1800;
    if (mapped < 200)  mapped = 200;

    int32_t offset = mapped - 1000;

    /* 中位死区 (在映射后的值域上判断，±50 对应摇杆行程约 ±6%) */
    if (offset > -JOYSTICK_DEADBAND && offset < JOYSTICK_DEADBAND) offset = 0;

    return (int16_t)(offset + 1000);
}

/* ---------------------------------------------------------------------------*/
/*  环形缓冲拷贝重组                                                            */
/* ---------------------------------------------------------------------------*/
static void SBUS_CopyFrameFromRing(uint8_t *dst)
{
    if (dst == NULL) return;

    uint16_t remaining = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
    uint16_t nextIndex = (SBUS_FRAME_SIZE - remaining) % SBUS_FRAME_SIZE;
    uint16_t firstChunk = SBUS_FRAME_SIZE - nextIndex;

    memcpy(dst, &SBUS_RxBuffer[nextIndex], firstChunk);
    if (nextIndex > 0)
    {
        memcpy(dst + firstChunk, SBUS_RxBuffer, nextIndex);
    }
}

/* ---------------------------------------------------------------------------*/
/*  失控保护                                                                    */
/* ---------------------------------------------------------------------------*/
static void RC_Failsafe(void)
{
    rc_data.right_x = 1000;
    rc_data.right_y = 1000;
    rc_data.left_y  = 1000;
    rc_data.left_x  = 1000;
    rc_data.ch5     = 1000;
    rc_data.ch6     = 1000;
    rc_data.ch7     = 1000;
    rc_data.ch8     = 1000;
    rc_data.ch9     = 1000;
    rc_data.ch10    = 1000;
    rc_data.connected = 0;
}

/* ---------------------------------------------------------------------------*/
/*  10通道完整解码核心                                                          */
/*  SBUS 协议每个通道占 11 bit，通过位移进行拼接                                  */
/* ---------------------------------------------------------------------------*/
static uint8_t sbus_decode(uint8_t *buf)
{
    /* 1. 检查头尾标志是否合法 (0x0F 打头, 0x00 结尾) */
    if (buf == NULL || buf[0] != 0x0F || buf[24] != 0x00) return 0;

    /* 2. 检查 Frame Lost (失联) 标志位 */
    /* 如果遥控器关机或信号丢失，接收机会把 buf[23] 的第3位置1 (即 0x0C) */
    if (buf[23] & 0x0C)
    {
        RC_Failsafe();
        return 0;
    }

    /* 3. 位运算拼接 1-10 通道 (11位/通道) */
    uint16_t ch1  = ((buf[1]       | buf[2] << 8)                 & 0x07FF);
    uint16_t ch2  = ((buf[2] >> 3  | buf[3] << 5)                 & 0x07FF);
    uint16_t ch3  = ((buf[3] >> 6  | buf[4] << 2 | buf[5] << 10)  & 0x07FF);
    uint16_t ch4  = ((buf[5] >> 1  | buf[6] << 7)                 & 0x07FF);
    uint16_t ch5  = ((buf[6] >> 4  | buf[7] << 4)                 & 0x07FF);
    uint16_t ch6  = ((buf[7] >> 7  | buf[8] << 1 | buf[9] << 9)   & 0x07FF);
    uint16_t ch7  = ((buf[9] >> 2  | buf[10] << 6)                & 0x07FF);
    uint16_t ch8  = ((buf[10] >> 5 | buf[11] << 3)                & 0x07FF);
    uint16_t ch9  = ((buf[12]      | buf[13] << 8)                & 0x07FF);
    uint16_t ch10 = ((buf[13] >> 3 | buf[14] << 5)                & 0x07FF);

    /* 4. 摇杆赋值 (映射到 200~1800，中位 1000) */
    rc_data.right_x = sbus_to_rc(ch1);  /* CH1 */
    rc_data.left_y  = sbus_to_rc(ch2);  /* CH2 */
    rc_data.right_y = sbus_to_rc(ch3);  /* CH3 */
    rc_data.left_x  = sbus_to_rc(ch4);  /* CH4 */

    /* 5. 拨杆/旋钮辅助通道赋值 */
    rc_data.ch5  = sbus_to_rc(ch5);
    rc_data.ch6  = sbus_to_rc(ch6);
    rc_data.ch7  = sbus_to_rc(ch7);
    rc_data.ch8  = sbus_to_rc(ch8);
    rc_data.ch9  = sbus_to_rc(ch9);
    rc_data.ch10 = sbus_to_rc(ch10);

    /* 6. 标记状态为已连接 */
    rc_data.connected = 1;
    return 1;
}

/* ---------------------------------------------------------------------------*/
/*  USART3 空闲中断处理函数                                                     */
/*  需要在 stm32f4xx_it.c 的 USART3_IRQHandler 中调用此函数！                    */
/* ---------------------------------------------------------------------------*/
void RC_UART_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET &&
        __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        SBUS_CopyFrameFromRing(sbus_data);

        if (sbus_decode(sbus_data)) {
            last_valid_frame_tick = HAL_GetTick();
        }
    }
}
