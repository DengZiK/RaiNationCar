#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

typedef enum {
    SYS_STATUS_OK = 0,
    SYS_STATUS_REMOTE_LOST,
    SYS_STATUS_MOTOR_TIMEOUT,
    SYS_STATUS_ESTOP,
    SYS_STATUS_TASK_STALL,
} System_Status_t;

typedef enum {
    WATCHDOG_TASK_REMOTE = 0,
    WATCHDOG_TASK_CHASSIS,
    WATCHDOG_TASK_LIFT,
    WATCHDOG_TASK_VALVE,
    WATCHDOG_TASK_COUNT,
} Watchdog_TaskId_t;

/* ========================================================================= */
/*  故障码 — 系统异常时记录, 用于"死机"后定位                                  */
/*                                                                            */
/*  记录方式:                                                                  */
/*    1. g_fault_code (RAM):   死机瞬间的值, 接调试器可直接读取                 */
/*    2. RTC 备份寄存器 BKP0R:  跨复位保留, IWDG 自动复位后仍可读取              */
/*  读取方式:                                                                  */
/*    1. 调试器 Watch 窗口看 g_fault_code                                      */
/*    2. 复位后开机 Watchdog_Init 会用 LED 闪烁 (code+1) 次提示上次故障类型     */
/* ========================================================================= */
#define FAULT_NONE            0U   /**< 无故障 */
#define FAULT_STACK_OVERFLOW  1U   /**< 任务栈溢出 */
#define FAULT_HARDFAULT       2U   /**< HardFault */
#define FAULT_MEM_MANAGE      3U   /**< MemManage Fault (内存管理) */
#define FAULT_BUS_FAULT       4U   /**< BusFault (总线错误) */
#define FAULT_USAGE_FAULT     5U   /**< UsageFault (用法错误) */
#define FAULT_ASSERT          6U   /**< FreeRTOS configASSERT 触发 */
#define FAULT_MALLOC_FAILED   7U   /**< FreeRTOS 堆分配失败 */
#define FAULT_HAL_ERROR       8U   /**< HAL Error_Handler */

/** @brief 故障码全局变量 — 定义于 watchdog.c, 供各故障入口写入 */
extern volatile uint32_t g_fault_code;
extern volatile uint32_t g_stack_overflow_flag;

void Watchdog_Init(void);
void Watchdog_Update(void);
System_Status_t Watchdog_GetStatus(void);

/** @brief 记录故障 (RAM + 备份寄存器), 供故障处理器/钩子调用 */
void Fault_Record(uint32_t code);

/** @brief 启动硬件独立看门狗 (IWDG) — 需在系统初始化完成后调用 */
void Watchdog_StartIWDG(void);

/** @brief 喂硬件看门狗 — 由安全监控任务周期性调用 */
void Watchdog_Feed(void);
void Watchdog_TaskHeartbeat(Watchdog_TaskId_t task);

#ifdef __cplusplus
}
#endif

#endif /* __WATCHDOG_H__ */
