#ifndef __LIFT_H__
#define __LIFT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "pid.h"

/*
 * 预设挡位 LIFT_POS_UP / LIFT_POS_DOWN 在 app_config.h 中定义 (以 mm 为单位),
 * 与软限位 LIFT_POSITION_MIN / LIFT_POSITION_MAX 一并集中管理 (参数单页)。
 * 本文件只保留使用说明, 不再重复定义。
 *
 * 拨杆上 → 最高挡位 = LIFT_POS_UP_MM 480mm (≈ 大电机 2.41 圈)
 * 拨杆下 → 最低点 = LIFT_POSITION_MIN (大电机 1/4 圈 ≈ 49.7mm)
 *
 * 最高挡位 < 软限位上限 LIFT_POSITION_MAX_MM 596.73mm (≈ 大电机 3 圈):
 *   挡位 480mm 距软限位 ~117mm, CH2 微调可上探而不顶硬限位。
 */

/*
 * 到位判定公差 (编码器计数) — 带滞回, 防状态在到位点附近抖动
 *   |误差| ≤ ARRIVED → 判定到位 (HOLD)
 *   离开 HOLD 需 |误差| > LEAVE (滞回带, 防止边界反复切换)
 */
#define LIFT_ARRIVED_TOLERANCE   200U  /**< 到位判定阈值 */
#define LIFT_LEAVE_TOLERANCE     500U  /**< 离开 HOLD 阈值 (防抖) */

typedef enum {
    LIFT_STATE_IDLE = 0,
    LIFT_STATE_MOVING_UP,
    LIFT_STATE_MOVING_DOWN,
    LIFT_STATE_HOLD,
    LIFT_STATE_ESTOP,
} Lift_State_t;

typedef struct {
    Lift_State_t state;

    float target_position;      /* 目标位置 (编码器计数) */
    float current_position;     /* 当前位置 (编码器计数) */
    float target_speed;         /* 中间量: 位置环输出 = 速度指令 (rpm) */
    float current_speed;        /* 当前转速 (rpm, 来自反馈) */

    PID_TypeDef pos_pid;        /* 外环: 位置 → 速度指令 */
    PID_TypeDef spd_pid;        /* 内环: 速度 → 电流指令 */

    uint8_t  last_switch_state;
    uint8_t  fine_adjust;       /* 1 = CH2 左摇杆正推杆微调升降 (三档都可用, 底盘禁止自转) */
    uint32_t loop_count;
} Lift_Handle_t;

extern Lift_Handle_t g_lift;

void Lift_Init(void);
void Lift_Update(void);
void Lift_EmergencyStop(void);
Lift_State_t Lift_GetState(void);
float Lift_GetHeightMM(void);

/**
 * @brief   查询 CH2 是否正被推杆用于升降微调
 * @retval  1 = CH2 (left_y) 摇杆正被推杆, 微调升降中 (底盘禁止自转)
 *          0 = 松杆 / 未使用微调
 * @note    三个拨杆档位均会触发; 供底盘模块调用: 微调期间左摇杆被 CH2
 *          占用, 禁止底盘旋转 (含航向保持)
 */
uint8_t Lift_IsFineAdjust(void);

#ifdef __cplusplus
}
#endif

#endif /* __LIFT_H__ */
