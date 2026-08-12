#ifndef REMOTE_H__
#define REMOTE_H__

#include "stm32f4xx_hal.h"

/* SBUS 协议参数 */
#define SBUS_FRAME_SIZE     25
#define SBUS_TIMEOUT_MS      100U

/* 拨片参数宏定义 */
#define RC_SW_UP            200
#define RC_SW_MID           1000
#define RC_SW_DOWN          1800

/* AT9S 十通道全解析结构体 */
typedef struct
{
    /* 基础摇杆通道 (映射范围: 200 ~ 1800，中位1000)
     * 死区不再在本层处理, 统一交由应用层 app_config.h 各通道死区宏控制 */
    int16_t right_x;    /* CH1: 右摇杆左右 */
    int16_t right_y;    /* CH2: 右摇杆上下 */
    int16_t left_y;     /* CH3: 左摇杆上下 */
    int16_t left_x;     /* CH4: 左摇杆左右 */

    /* 辅助通道 CH5 - CH10 (映射范围: 200 ~ 1800，中位1000) */
    int16_t ch5;        /* Sa */
    int16_t ch6;        /* Sb */
    int16_t ch7;        /* Sc */
    int16_t ch8;        /* Sd */
    int16_t ch9;        /* Se */
    int16_t ch10;       /* Sg */

    /* 连接状态保护 */
    uint8_t connected;  /* 1: 正常连接, 0: 信号丢失 (失控保护) */

} RC_Data_t;

extern RC_Data_t rc_data;

/* 全局数组声明，方便 Debug 窗口直接查看底层数据 */
extern uint8_t SBUS_RxBuffer[SBUS_FRAME_SIZE];
extern uint8_t sbus_data[SBUS_FRAME_SIZE];

/* 核心函数声明 */
HAL_StatusTypeDef remote_control_init(void);     /* 遥控器初始化 */
void remote_control_watchdog(void);              /* 遥控器超时检测，需周期调用 */
void RC_UART_IRQHandler(void);                   /* 串口中断函数 */
uint32_t remote_control_frame_age(void);         /* 距上次有效遥控帧的毫秒数 (诊断用) */

#endif /* REMOTE_H__ */
