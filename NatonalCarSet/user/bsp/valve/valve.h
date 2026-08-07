#ifndef __VALVE_H__
#define __VALVE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"

/*
 * 阀编号 — 与 app_config.h 中 VALVE_1~VALVE_4 按顺序一一对应 (0 起)
 * 数组大小/循环上界使用 VALVE_COUNT 宏 (app_config.h)
 */
typedef enum {
    VALVE_1 = 0,
    VALVE_2,
    VALVE_3,
    VALVE_4,
} Valve_t;

typedef enum {
    VALVE_STATE_OFF = 0,
    VALVE_STATE_ON  = 1,
} Valve_State_t;

void Valve_Init(void);
void Valve_On(Valve_t valve);
void Valve_Off(Valve_t valve);
void Valve_Toggle(Valve_t valve);
Valve_State_t Valve_GetState(Valve_t valve);
void Valve_AllOff(void);
void Valve_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __VALVE_H__ */
