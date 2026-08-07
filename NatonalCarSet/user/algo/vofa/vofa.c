/**
 * @file    vofa.c
 * @brief   VOFA+ JustFloat 协议实现
 *
 * @note    Arduino 参考:
 *            Serial.write((char*)data, sizeof(float)*CH_COUNT);
 *            char tail[4] = {0x00,0x00,0x80,0x7f};
 *            Serial.write(tail, 4);
 *
 *          HAL 阻塞发送 ≈ (count×4+4) × 10bit ÷ 115200
 *          2ch ≈ 1.04ms,  4ch ≈ 1.74ms,  8ch ≈ 3.12ms
 ******************************************************************************
 */

#include "vofa.h"
#include <string.h>

extern UART_HandleTypeDef huart6;

/*
 * 帧尾 = float +Inf (IEEE 754 LE)
 *
 *   uint8_t  tail[4] = {0x00, 0x00, 0x80, 0x7f};
 *            └─ 小端字节序: 0x7F800000 = 0_11111111_000...0 = +Inf
 *
 *   参考: https://www.vofa.plus/docs/learning/dataengines/justfloat/
 */
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};

/* ---------------------------------------------------------------------------*/
/*  VOFA_Send — 发送 N 通道浮点帧 (阻塞, 10ms 超时)                            */
/* ---------------------------------------------------------------------------*/
void VOFA_Send(const float *data, uint8_t count)
{
    if (data == NULL || count == 0 || count > VOFA_MAX_CH) return;

    /*
     * 发送浮点数组 (小端, 直接内存拷贝)
     *
     *   STM32F4 硬件 FPU 使用 IEEE 754 LE 格式, 与 VOFA+ (x86) 一致,
     *   无需字节序转换。如果移植到 51 等大端平台, 需逐通道翻转字节序。
     */
    (void)HAL_UART_Transmit(&huart6, (const uint8_t *)data,
                            count * sizeof(float), 10);

    /*
     * 发送帧尾 — VOFA+ 收到 {0x00,0x00,0x80,0x7f} 后立即解析整帧
     */
    (void)HAL_UART_Transmit(&huart6, tail, 4, 10);
}

/* ---------------------------------------------------------------------------*/
/*  VOFA_SendStatus — 死机诊断状态帧 (文本, 串口助手可读)                       */
/*                                                                              */
/*  输出示例:  T=123456 F=0 C=1 S=O O=0 A=12 M=00000\r\n                       */
/*    T  = HAL_GetTick()   — 死机后不变 = 中断被关/进 fault                     */
/*    F  = g_fault_code    — 0=无故障, 1~8=见 watchdog.h                        */
/*    C  = rc_data.connected — 遥控是否在收                                     */
/*    S  = 系统状态 O=OK / L=遥控失联 / M=电机超时 / E=急停                     */
/*    O  = 栈溢出标志                                                            */
/*    A  = 距上次有效遥控帧的毫秒数 (遥控链路年龄)                                */
/*    M  = 5 位电机超时位 (M0~M4, 1=超时)                                        */
/* ---------------------------------------------------------------------------*/

/* 无符号整数 → 十进制, 追加到缓冲区 (无 printf 依赖, 控制栈占用) */
static char *u32_append(char *p, uint32_t v)
{
    char tmp[10];
    uint8_t n = 0;

    if (v == 0U) {
        *p++ = '0';
        return p;
    }
    while (v > 0U) {
        tmp[n++] = (char)('0' + (v % 10U));
        v /= 10U;
    }
    while (n > 0U) {
        *p++ = tmp[--n];
    }
    return p;
}

void VOFA_SendStatus(const VOFA_Status_t *st)
{
    char line[64];
    char *p = line;
    char status_ch;

    if (st == NULL) return;

    /* 系统状态 → 单字母 */
    switch (st->sys_status) {
        case 1:  status_ch = 'L'; break;   /* 遥控失联 */
        case 2:  status_ch = 'M'; break;   /* 电机超时 */
        case 3:  status_ch = 'E'; break;   /* 急停 */
        default: status_ch = 'O'; break;   /* OK */
    }

    *p++ = 'T'; *p++ = '='; p = u32_append(p, st->tick);
    *p++ = ' '; *p++ = 'F'; *p++ = '='; p = u32_append(p, st->fault_code);
    *p++ = ' '; *p++ = 'C'; *p++ = '='; *p++ = (char)('0' + (st->connected ? 1 : 0));
    *p++ = ' '; *p++ = 'S'; *p++ = '='; *p++ = status_ch;
    *p++ = ' '; *p++ = 'O'; *p++ = '='; *p++ = (char)('0' + (st->stack_overflow ? 1 : 0));
    *p++ = ' '; *p++ = 'A'; *p++ = '='; p = u32_append(p, st->remote_age_ms);
    *p++ = ' '; *p++ = 'M'; *p++ = '=';
    for (uint8_t i = 0; i < 5U; i++) {
        *p++ = (char)('0' + (st->motor_timeout[i] ? 1 : 0));
    }
    *p++ = '\r';
    *p++ = '\n';

    (void)HAL_UART_Transmit(&huart6, (const uint8_t *)line,
                            (uint16_t)(p - line), 10);
}
