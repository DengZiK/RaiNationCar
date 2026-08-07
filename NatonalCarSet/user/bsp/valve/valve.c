#include "valve.h"
#include "remote.h"

/* 每个阀的引脚 + 遥控器通道配置 (与 app_config.h 中 VALVE_x_PIN/RC_CHANNEL 对应) */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
    uint8_t       rc_channel;   /* 遥控器辅助通道号 (CH5~CH10) */
} Valve_Config_t;

static const Valve_Config_t g_valve_cfg[VALVE_COUNT] = {
    { VALVE_GPIO_PORT, VALVE_1_PIN, VALVE_1_RC_CHANNEL },
    { VALVE_GPIO_PORT, VALVE_2_PIN, VALVE_2_RC_CHANNEL },
    { VALVE_GPIO_PORT, VALVE_3_PIN, VALVE_3_RC_CHANNEL },
    { VALVE_GPIO_PORT, VALVE_4_PIN, VALVE_4_RC_CHANNEL },
};

static Valve_State_t g_valve_state[VALVE_COUNT];
static uint8_t       g_last_sw_state[VALVE_COUNT];   /* 边沿检测: 0=UP, 1=MID, 2=DOWN */

/*
 * 按遥控器通道号读取原始值 (只支持辅助通道 CH5~CH10)
 * 映射后值域: 200~1800, 中位 1000
 */
static int16_t Rc_GetChannel(uint8_t ch)
{
    switch (ch) {
        case 5:  return rc_data.ch5;
        case 6:  return rc_data.ch6;
        case 7:  return rc_data.ch7;
        case 8:  return rc_data.ch8;
        case 9:  return rc_data.ch9;
        case 10: return rc_data.ch10;
        default: return 1000;
    }
}

void Valve_Init(void)
{
    for (uint8_t i = 0; i < VALVE_COUNT; i++) {
        HAL_GPIO_WritePin(g_valve_cfg[i].port, g_valve_cfg[i].pin, GPIO_PIN_RESET);
        g_valve_state[i]   = VALVE_STATE_OFF;
        g_last_sw_state[i] = 0xFF;   /* 强制首周期按拨杆位置动作一次 */
    }
}

void Valve_On(Valve_t valve)
{
    if (valve >= VALVE_COUNT) {
        return;
    }
    HAL_GPIO_WritePin(g_valve_cfg[valve].port, g_valve_cfg[valve].pin, GPIO_PIN_SET);
    g_valve_state[valve] = VALVE_STATE_ON;
}

void Valve_Off(Valve_t valve)
{
    if (valve >= VALVE_COUNT) {
        return;
    }
    HAL_GPIO_WritePin(g_valve_cfg[valve].port, g_valve_cfg[valve].pin, GPIO_PIN_RESET);
    g_valve_state[valve] = VALVE_STATE_OFF;
}

void Valve_Toggle(Valve_t valve)
{
    if (valve >= VALVE_COUNT) {
        return;
    }
    if (g_valve_state[valve] == VALVE_STATE_ON) {
        Valve_Off(valve);
    } else {
        Valve_On(valve);
    }
}

Valve_State_t Valve_GetState(Valve_t valve)
{
    if (valve >= VALVE_COUNT) {
        return VALVE_STATE_OFF;
    }
    return g_valve_state[valve];
}

/* 急停/失控时一键关闭全部阀 — 供 watchdog 调用 */
void Valve_AllOff(void)
{
    for (uint8_t i = 0; i < VALVE_COUNT; i++) {
        Valve_Off((Valve_t)i);
    }
}

/*
 * 电磁阀控制 — 由任务循环调用 (50Hz)
 *   4 路阀各自独立, 由对应遥控器拨杆控制:
 *     DOWN (>1500) → 开阀
 *     UP   (<500)  → 关阀
 *     MID (中位)   → 保持当前状态 (三档开关可做"保持", 不动作)
 *   仅拨杆状态变化时才写 GPIO, 防反复触发
 */
void Valve_Process(void)
{
    /* DEBUG: PE0 心跳 — 每进一次翻转 (25Hz方波 = 任务在跑) */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_0);

    for (uint8_t i = 0; i < VALVE_COUNT; i++) {
        int16_t ch_val = Rc_GetChannel(g_valve_cfg[i].rc_channel);

        uint8_t sw_state;
        if (ch_val > 1500) {
            sw_state = 2;   /* DOWN */
        } else if (ch_val < 500) {
            sw_state = 0;   /* UP */
        } else {
            sw_state = 1;   /* MID */
        }

        if (sw_state != g_last_sw_state[i]) {
            g_last_sw_state[i] = sw_state;
            if (sw_state == 2) {
                Valve_On((Valve_t)i);
            } else if (sw_state == 0) {
                Valve_Off((Valve_t)i);
            }
            /* MID: 保持当前状态, 不动作 */
        }
    }
}
