#include "chassis.h"
#include "motor.h"
#include "remote.h"
#include "imu.h"
#include "lift.h"
#include <math.h>

Chassis_Handle_t g_chassis;

void Chassis_Init(void)
{
    memset(&g_chassis, 0, sizeof(g_chassis));
    g_chassis.state = CHASSIS_STATE_IDLE;

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        PID_Init(&g_chassis.wheel_pid[i],
                 CHASSIS_SPEED_KP,
                 CHASSIS_SPEED_KI,
                 CHASSIS_SPEED_KD,
                 CHASSIS_SPEED_MAX_OUT,
                 CHASSIS_SPEED_MAX_I,
                 CHASSIS_SPEED_SEP_THRESH,
                 CHASSIS_SPEED_D_ALPHA);
    }

    /* 航向外环 PID (航向角→Wz 角速度指令) */
    PID_Init(&g_chassis.heading_pid,
             HEADING_PID_KP,
             HEADING_PID_KI,
             HEADING_PID_KD,
             HEADING_PID_MAX_OUT,
             HEADING_PID_MAX_I,
             HEADING_PID_SEP_THRESH,
             HEADING_PID_D_ALPHA);
}

/*
 * 麦科勒姆逆运动学
 *   motor[0]=FR, motor[1]=FL, motor[2]=RL, motor[3]=RR
 *
 *   基础公式 (电机转子正方向):
 *     V_FR =  Vx - Vy - K*Wz
 *     V_FL =  Vx + Vy + K*Wz
 *     V_RL = -Vx + Vy - K*Wz
 *     V_RR = -Vx - Vy + K*Wz
 *     K = Lx + Ly
 *
 *   motor_dir[] 补偿物理安装朝向差异:
 *     1.0 = 正常, -1.0 = 电机翻转180°安装 (镜像布局)
 *     当前: FL(2号电机) 与 FR(1号电机) 安装方向相反 → motor_dir[1] = -1.0
 *           RR(4号电机) 转向反 → motor_dir[3] = -1.0
 */
void Chassis_MecanumKinematics(float vx, float vy, float wz, float rpm_out[4])
{
    float K = CHASSIS_LX + CHASSIS_LY;

    /* 电机物理安装方向补偿 (1.0=正常, -1.0=翻转) */
    const float motor_dir[4] = {1.0f, -1.0f, 1.0f, -1.0f};

    float v_wheel[4];
    v_wheel[0] = motor_dir[0] * ( vx - vy - K * wz);   /* FR */
    v_wheel[1] = motor_dir[1] * ( vx + vy + K * wz);   /* FL — 物理翻转, 取反 */
    v_wheel[2] = motor_dir[2] * (-vx + vy - K * wz);   /* RL */
    v_wheel[3] = motor_dir[3] * (-vx - vy + K * wz);   /* RR */

    for (uint8_t i = 0; i < 4; i++) {
        rpm_out[i] = v_wheel[i] * WHEEL_MMPS_TO_RPM * CHASSIS_RPM_SCALE;
    }
}

/*
 * 速度指令限速 (加速度/减速度分离)
 *
 *   current     当前平滑指令
 *   desired     摇杆直接解算的目标
 *   accel_step  每控制周期最大加速增量
 *   decel_step  每控制周期最大减速增量
 *
 * 规则: 同向且目标幅值更大 → 按加速斜率逼近; 其余 (减速/反向) → 按减速斜率。
 * 作用: 松杆回中时目标按减速斜率线性降到 0, 不再瞬间归零刹死。
 */
static float Chassis_RateLimit(float current, float desired, float accel_step, float decel_step)
{
    float diff = desired - current;
    float step = (desired * current >= 0.0f && fabsf(desired) >= fabsf(current))
                 ? accel_step : decel_step;

    if (diff >  step) return current + step;
    if (diff < -step) return current - step;
    return desired;
}

/*
 * 底盘控制主循环 (200Hz)
 *   遥控器映射 (AT9S Mode 2):
 *     right_y (CH3): 前/后 → Vx
 *     right_x (CH1): 左/右 → Vy
 *     left_x  (CH4): 旋转 → Wz
 */
void Chassis_Update(void)
{
    g_chassis.loop_count++;

    /* 安全检查 */
    if (!rc_data.connected) {
        Chassis_EmergencyStop();
        return;
    }

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        if (Motor_IsTimeout(i, CAN_MOTOR_TIMEOUT_MS)) {
            Chassis_EmergencyStop();
            return;
        }
    }

    /* 0. IMU 更新 (200Hz) — 陀螺仪航向积分, 必须最先 */
    IMU_Update();

    /* 1. 读取遥控器 → 目标速度 (SBUS: 200~1800, 中位1000) */
    float norm_vx = (float)(rc_data.right_y - 1000) / 800.0f * RC_JOYSTICK_SCALE;
    float norm_vy = (float)(rc_data.right_x - 1000) / 800.0f * RC_JOYSTICK_SCALE;
    float norm_wz = (float)(rc_data.left_x  - 1000) / 800.0f * RC_JOYSTICK_SCALE;

    /* 死区 — 前进/平移通道 (Vx/Vy) 用固定小死区 */
    if (fabsf(norm_vx) < 0.02f) norm_vx = 0.0f;
    if (fabsf(norm_vy) < 0.02f) norm_vy = 0.0f;

    /* 死区 — 旋转通道 (CH4): 出死区后输出从 0 起线性增大 (满杆仍到满幅),
     * 避免进入/退出死区瞬间 Wz 从 0 跳变到死区边界值。
     * (误触旋转已由加大 LIFT_FINE_DEADBAND + 微调期间强制 Wz=0 抑制) */
    if (fabsf(norm_wz) <= CHASSIS_ROT_DEADBAND) {
        norm_wz = 0.0f;
    } else {
        float wz_abs  = fabsf(norm_wz);
        float wz_max  = RC_JOYSTICK_SCALE;                       /* 满杆 |norm_wz| 上限 */
        float wz_sign = (norm_wz > 0.0f) ? 1.0f : -1.0f;
        norm_wz = wz_sign * (wz_abs - CHASSIS_ROT_DEADBAND) / (wz_max - CHASSIS_ROT_DEADBAND);
    }

    float max_linear_speed  = CHASSIS_MAX_LINEAR_SPEED_MMPS;
    float max_angular_speed = CHASSIS_MAX_ANGULAR_SPEED_RADPS;

    g_chassis.target_vx_mmps  = norm_vx * max_linear_speed;
    /* 平移方向: 右杆打向与实际横移方向相反, 取反修正 */
    /* 横移速度单独缩放: 左右平移比前后慢 20% */
    g_chassis.target_vy_mmps  = -norm_vy * max_linear_speed * CHASSIS_STRAFE_SCALE;

    /* 2. 陀螺仪航向保持 — CH10(Sg 拨杆): DOWN(>1500)=开, 其余=关 (边沿检测) */
    uint8_t gyro_mode = (rc_data.ch10 > 1500) ? 1U : 0U;
    if (gyro_mode && !g_chassis.heading_mode_last) {
        /* OFF→ON 上升沿: 以当前航向为参考, 清 PID 防跳变 */
        PID_Clear(&g_chassis.heading_pid);
        g_chassis.heading_ref = g_imu.yaw;
    }
    if (!gyro_mode && g_chassis.heading_mode_last) {
        PID_Clear(&g_chassis.heading_pid);   /* ON→OFF 下降沿 */
    }
    g_chassis.heading_mode      = gyro_mode;
    g_chassis.heading_mode_last = gyro_mode;

    /* 3. Wz — 摇杆旋转优先 / 松杆回中保持 */
    uint8_t is_rotating = (fabsf(norm_wz) >= HEADING_WZ_DEADZONE);

    if (Lift_IsFineAdjust()) {
        /* 升降微调模式 (CH9 回中): 左摇杆已被 CH2 占用调升降,
         * 禁止底盘自转 (含航向保持)。
         * 同时把航向参考冻结为当前 yaw, 并清空航向 PID 积分,
         * 保证退出微调时航向保持不会突然转正。 */
        g_chassis.target_wz_radps = 0.0f;
        g_chassis.heading_ref     = g_imu.yaw;
        if (g_chassis.heading_mode) {
            PID_Clear(&g_chassis.heading_pid);
        }
    } else if (g_chassis.heading_mode && !is_rotating) {
        /* 松杆回中 → 航向保持 PID 接管 Wz (最短角差, 跨 ±180° 走捷径) */
        float yaw_err = IMU_AngleDiff(g_chassis.heading_ref, g_imu.yaw);
        /* PID_Calc err = 0 - (-yaw_err) = yaw_err = heading_ref - yaw */
        g_chassis.target_wz_radps = PID_Calc(&g_chassis.heading_pid, 0.0f, -yaw_err);
    } else {
        /* 旋转方向: 左杆打向某侧与实际自转方向相反, 取反修正 */
        g_chassis.target_wz_radps = -norm_wz * max_angular_speed;
        if (g_chassis.heading_mode && is_rotating) {
            if (!g_chassis.heading_rot_last) {
                PID_Clear(&g_chassis.heading_pid);   /* 刚进入旋转, 清积分防饱和 */
            }
            g_chassis.heading_ref = g_imu.yaw;       /* 旋转中持续刷新参考航向 */
        }
    }
    g_chassis.heading_rot_last = is_rotating;

    /* 4. 速度指令限速 — 松杆回中平滑减速, 不直接刹死 */
    float dt = 1.0f / CHASSIS_CONTROL_FREQ_HZ;

    g_chassis.cmd_vx_mmps = Chassis_RateLimit(g_chassis.cmd_vx_mmps,
                                              g_chassis.target_vx_mmps,
                                              CHASSIS_ACCEL_LIMIT_MMPS2 * dt,
                                              CHASSIS_DECEL_LIMIT_MMPS2 * dt);
    g_chassis.cmd_vy_mmps = Chassis_RateLimit(g_chassis.cmd_vy_mmps,
                                              g_chassis.target_vy_mmps,
                                              CHASSIS_ACCEL_LIMIT_MMPS2 * dt,
                                              CHASSIS_DECEL_LIMIT_MMPS2 * dt);
    g_chassis.cmd_wz_radps = Chassis_RateLimit(g_chassis.cmd_wz_radps,
                                               g_chassis.target_wz_radps,
                                               CHASSIS_ANG_ACCEL_LIMIT_RADPS2 * dt,
                                               CHASSIS_ANG_DECEL_LIMIT_RADPS2 * dt);

    /* 5. 平滑指令已≈0 → 真正停稳才算 IDLE (减速过程仍为 MANUAL) */
    int is_idle = (fabsf(g_chassis.cmd_vx_mmps) < 1.0f &&
                   fabsf(g_chassis.cmd_vy_mmps) < 1.0f &&
                   fabsf(g_chassis.cmd_wz_radps) < 0.01f);

    if (is_idle) {
        g_chassis.state = CHASSIS_STATE_IDLE;
        /*
         * 目标转速 = 0，但不清除 PID 积分。
         * 这样外力推电机偏离 0 速时，I 项会逐步累积反向力矩，实现自锁。
         * 如果不需要自锁（想自由滑行），把 PID_Clear 加回来即可。
         */
    } else {
        g_chassis.state = CHASSIS_STATE_MANUAL;
    }

    /* 2. 逆运动学解算 (使用限速后的平滑指令) */
    Chassis_MecanumKinematics(g_chassis.cmd_vx_mmps,
                               g_chassis.cmd_vy_mmps,
                               g_chassis.cmd_wz_radps,
                               g_chassis.wheel_rpm_target);

    /* 3. 速度 PID */
    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        float feedback_rpm = (float)g_motor.feedback[i].speed_rpm;
        float current_out = PID_Calc(&g_chassis.wheel_pid[i],
                                      g_chassis.wheel_rpm_target[i],
                                      feedback_rpm);

        if (current_out >  C620_CURRENT_MAX) current_out =  C620_CURRENT_MAX;
        if (current_out < -C620_CURRENT_MAX) current_out = -C620_CURRENT_MAX;

        /* 仅急停时强制清零, IDLE 和 MANUAL 都允许 PID 输出 */
        if (g_chassis.state == CHASSIS_STATE_ESTOP) {
            current_out = 0.0f;
        }

        g_motor.current_setpoint[i] = (int16_t)current_out;
    }

    /* 4. CAN 发送 */
    Motor_SendChassisCurrent();
}

void Chassis_EmergencyStop(void)
{
    g_chassis.state = CHASSIS_STATE_ESTOP;
    g_chassis.estop_count++;

    for (uint8_t i = 0; i < CHASSIS_MOTOR_COUNT; i++) {
        g_motor.current_setpoint[i] = 0;
        g_chassis.wheel_rpm_target[i] = 0.0f;
        PID_Clear(&g_chassis.wheel_pid[i]);
    }

    /* 急停同时清零平滑指令, 防止恢复后带着旧速度起步 */
    g_chassis.cmd_vx_mmps = 0.0f;
    g_chassis.cmd_vy_mmps = 0.0f;
    g_chassis.cmd_wz_radps = 0.0f;

    Motor_SendChassisCurrent();
}

Chassis_State_t Chassis_GetState(void)
{
    return g_chassis.state;
}
