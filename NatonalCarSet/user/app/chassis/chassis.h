#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app_config.h"
#include "pid.h"

typedef enum {
    CHASSIS_STATE_IDLE = 0,
    CHASSIS_STATE_MANUAL,
    CHASSIS_STATE_ESTOP,
} Chassis_State_t;

typedef struct {
    Chassis_State_t state;

    /* 运动学目标 (来自遥控器, 摇杆直接解算) */
    float target_vx_mmps;
    float target_vy_mmps;
    float target_wz_radps;

    /* 限速后的平滑速度指令 (加速度/减速度分离, 松杆不刹死) */
    float cmd_vx_mmps;
    float cmd_vy_mmps;
    float cmd_wz_radps;

    /* 各轮目标转速 (rpm, 电机轴端) */
    float wheel_rpm_target[CHASSIS_MOTOR_COUNT];

    /* 速度 PID 控制器 (4轮独立) */
    PID_TypeDef wheel_pid[CHASSIS_MOTOR_COUNT];

    /* --- 陀螺仪航向保持 (CH10 开关) --- */
    PID_TypeDef heading_pid;        /* 航向外环 PID → Wz 角速度指令 */
    float       heading_ref;        /* 参考航向 (rad) */
    uint8_t     heading_mode;       /* 陀螺仪模式: 1=开, 0=关 */
    uint8_t     heading_mode_last;  /* 模式边沿检测 */
    uint8_t     heading_rot_last;   /* 摇杆旋转边沿检测 */

    uint32_t loop_count;
    uint32_t estop_count;
} Chassis_Handle_t;

extern Chassis_Handle_t g_chassis;

void Chassis_Init(void);
void Chassis_Update(void);
void Chassis_MecanumKinematics(float vx, float vy, float wz, float rpm_out[4]);
void Chassis_EmergencyStop(void);
Chassis_State_t Chassis_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
