#include "watchdog.h"
#include "remote.h"
#include "motor.h"
#include "chassis.h"
#include "lift.h"
#include "valve.h"
#include "vofa.h"

/* 栈溢出标志 — 定义于 freertos.c, 供死机诊断状态帧读取 */
volatile uint32_t g_stack_overflow_flag = 0;   /* set by vApplicationStackOverflowHook */

static System_Status_t g_sys_status = SYS_STATUS_OK;
static uint32_t        g_led_blink_tick = 0;
static uint8_t         g_led_state = 0;

/* 故障码全局变量 — 定义在此, 供 FreeRTOSConfig.h / 各 fault 入口 extern 引用 */
volatile uint32_t g_fault_code = FAULT_NONE;

/* ========================================================================= */
/*  RTC 备份寄存器 — 跨复位保留故障码                                         */
/*                                                                            */
/*  备份寄存器位于 VDD 备份域, 系统复位(IWDG复位)后内容保留,                   */
/*  因此可用于: 死机 → 记录故障码 → IWDG自动复位 → 上电后读取上次故障类型      */
/* ========================================================================= */
static void BKP_Write(uint32_t value)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;                      /* 解除备份域写保护 (DBP) */
    RTC->WPR = 0xCA;                            /* 解除 RTC 写保护 (1/2) */
    RTC->WPR = 0x53;                            /* 解除 RTC 写保护 (2/2) */
    RTC->BKP0R = value;                         /* 写入备份寄存器 0 */
    RTC->WPR = 0xFF;                            /* 重新上锁 */
}

static uint32_t BKP_Read(void)
{
    __HAL_RCC_PWR_CLK_ENABLE();
    PWR->CR |= PWR_CR_DBP;                      /* 备份域读也需 DBP */
    return RTC->BKP0R;
}

/*
 * 记录故障: 同时写入 RAM (调试器即时可读) 与备份寄存器 (复位后保留)。
 * 可在 fault 处理器/钩子中调用, 仅做寄存器访问, 不依赖 RTOS。
 */
void Fault_Record(uint32_t code)
{
    g_fault_code = code;
    BKP_Write(code);
}

/* ========================================================================= */
/*  硬件独立看门狗 (IWDG)                                                      */
/*                                                                            */
/*  直接寄存器操作 (不依赖 HAL 模块, 两种构建系统均可用)。                      */
/*  一旦启动, 若系统死机 (任何 while(1) 故障入口 / 调度器停止 /                */
/*  watchdog任务卡死), 计数器倒计时到 0 即自动复位, 不再依赖手动断电。          */
/*                                                                            */
/*  超时计算:  LSI ≈ 32kHz, PR=4 → /64 → 计数时钟 500Hz (2ms/计数)            */
/*            RLR = 500 → 超时 1s                                             */
/*            安全任务每 100ms 喂狗 → 余量 10×                                 */
/* ========================================================================= */
#define IWDG_ACCESS_KEY   0x5555U   /* 使能访问 PR/RLR */
#define IWDG_RELOAD_KEY   0xAAAAU   /* 喂狗 / 重装计数器 */
#define IWDG_START_KEY    0xCCCCU   /* 启动 IWDG */

void Watchdog_StartIWDG(void)
{
    uint32_t timeout;

    IWDG->KR = IWDG_ACCESS_KEY;                       /* 解除 PR/RLR 写保护 */

    timeout = 100000U;
    while ((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) {
        if (--timeout == 0U) break;                   /* 防卡死保护 */
    }

    IWDG->PR = 4U;                                    /* 预分频 /64 */
    timeout = 100000U;
    while ((IWDG->SR & IWDG_SR_PVU) != 0U) {
        if (--timeout == 0U) break;                   /* 等待 PR 更新完成 */
    }

    IWDG->RLR = 500U;                                 /* 重载值 → ~1s 超时 */
    timeout = 100000U;
    while ((IWDG->SR & IWDG_SR_RVU) != 0U) {
        if (--timeout == 0U) break;                   /* 等待 RLR 更新完成 */
    }

    IWDG->KR = IWDG_START_KEY;                        /* 启动 */
    IWDG->KR = IWDG_RELOAD_KEY;                       /* 首次喂狗 */
}

void Watchdog_Feed(void)
{
    IWDG->KR = IWDG_RELOAD_KEY;
}

void Watchdog_Init(void)
{
    g_sys_status = SYS_STATUS_OK;
    g_led_blink_tick = 0;
    g_led_state = 0;

    /*
     * 上次复位原因诊断:
     *   RCC->CSR.IWDGRSTF 置位 → 上次是硬件看门狗复位 → 之前发生过死机, 被自动拉回。
     *   配合备份寄存器里的故障码, 用 LED 闪烁 (code+1) 次提示故障类型, 便于定位。
     */
    uint32_t iwdg_reset = (RCC->CSR & RCC_CSR_IWDGRSTF) ? 1U : 0U;
    uint32_t last_fault = BKP_Read();

    /* 清除复位标志 + 备份寄存器, 避免下次上电误判 */
    RCC->CSR |= RCC_CSR_RMVF;
    BKP_Write(FAULT_NONE);

    if (iwdg_reset && (last_fault > FAULT_NONE) && (last_fault <= FAULT_HAL_ERROR)) {
        /* 故障恢复指示: 闪烁 (last_fault+1) 次, 每次间隔 200ms
         *   例: 栈溢出(1) → 闪2次; HardFault(2) → 闪3次; 以此类推 */
        for (uint32_t i = 0; i < (last_fault + 1U); i++) {
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
            HAL_Delay(200);
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
            HAL_Delay(200);
        }
    }

    /* LED 初始点亮 → 系统已上电 */
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
}

/*
 * 安全监控主循环 (10Hz / 100ms)
 */
void Watchdog_Update(void)
{
    System_Status_t new_status = SYS_STATUS_OK;

    /* 1. 调用遥控器看门狗 (更新 rc_data.connected) */
    remote_control_watchdog();

    /* 2. 检查遥控器连接 */
    if (!rc_data.connected) {
        new_status = SYS_STATUS_REMOTE_LOST;
        Chassis_EmergencyStop();
        Lift_EmergencyStop();
        Valve_AllOff();   /* 失控/急停: 关闭全部电磁阀 */
    }

    /* 3. 检查电机 CAN 通信 */
    uint8_t motor_fault = 0;
    uint8_t motor_to[MOTOR_COUNT] = {0};
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        motor_to[i] = Motor_IsTimeout(i, CAN_MOTOR_TIMEOUT_MS);
        if (motor_to[i]) {
            motor_fault = 1;
        }
    }

    if (motor_fault) {
        new_status = SYS_STATUS_MOTOR_TIMEOUT;
        Chassis_EmergencyStop();
        Lift_EmergencyStop();
    }

    if (new_status != SYS_STATUS_OK) {
        g_sys_status = new_status;
    } else if (g_sys_status != SYS_STATUS_ESTOP) {
        g_sys_status = SYS_STATUS_OK;
    }

    /* 4. LED 状态指示 */
    uint32_t now = HAL_GetTick();
    uint32_t blink_period;

    switch (g_sys_status) {
        case SYS_STATUS_OK:
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
            g_led_state = 1;
            break;

        case SYS_STATUS_REMOTE_LOST:
            blink_period = 500U;
            if ((now - g_led_blink_tick) >= blink_period) {
                g_led_blink_tick = now;
                g_led_state = !g_led_state;
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN,
                                  g_led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            break;

        case SYS_STATUS_MOTOR_TIMEOUT:
            blink_period = 200U;
            if ((now - g_led_blink_tick) >= blink_period) {
                g_led_blink_tick = now;
                g_led_state = !g_led_state;
                HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN,
                                  g_led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
            break;

        case SYS_STATUS_ESTOP:
            HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
            g_led_state = 0;
            break;
    }

    /* 喂硬件看门狗 — 本任务每 100ms 执行一次, 保证 IWDG 计数器存活。
     * 只要安全监控任务还在跑, IWDG 就不会复位; 一旦任务/内核死机, ~1s 后自动复位。 */
    Watchdog_Feed();

#if (VOFA_DIAG_MODE == 1)
    /* 死机诊断状态帧 (每周期一帧 = 100ms) — 串口助手看文本。
     * 放在喂狗之后, 即使发送阻塞也不会饿死 IWDG。
     * 死机后最后一行就是冻结前的状态:
     *   T 不变 = 真死机(fault), C=0/S=L = 遥控失联, S=M = 电机超时 */
    {
        VOFA_Status_t st;
        st.tick           = HAL_GetTick();
        st.fault_code     = g_fault_code;
        st.connected      = rc_data.connected;
        st.sys_status     = (uint8_t)g_sys_status;
        st.stack_overflow = (uint8_t)(g_stack_overflow_flag ? 1U : 0U);
        st.remote_age_ms  = remote_control_frame_age();
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            st.motor_timeout[i] = motor_to[i];
        }
        VOFA_SendStatus(&st);
    }
#endif /* VOFA_DIAG_MODE == 1 */
}

System_Status_t Watchdog_GetStatus(void)
{
    return g_sys_status;
}
