#include "lift.h"
#include "motor.h"
#include "remote.h"
#include <math.h>
#include <string.h>

Lift_Handle_t g_lift;

static const float g_preset_positions[3] = {
    [0] = (float)LIFT_POS_UP,     /* 拨杆上 → 最高挡位 (480mm ≈ 大电机 2.41 圈) */
    [2] = (float)LIFT_POS_DOWN,   /* 拨杆下 → 最低点 (软限位下限) */
    /* [1] 拨杆中 → 锁定当前位置, 不使用预设值 */
};

void Lift_Init(void)
{
    memset(&g_lift, 0, sizeof(g_lift));
    g_lift.state = LIFT_STATE_IDLE;

    PID_Init(&g_lift.pos_pid,
             LIFT_POS_KP,
             LIFT_POS_KI,
             LIFT_POS_KD,
             LIFT_POS_MAX_OUT,
             LIFT_POS_MAX_I,
             LIFT_POS_SEP_THRESH,
             LIFT_POS_D_ALPHA);

    PID_Init(&g_lift.spd_pid,
             LIFT_SPD_KP,
             LIFT_SPD_KI,
             LIFT_SPD_KD,
             LIFT_SPD_MAX_OUT,
             LIFT_SPD_MAX_I,
             LIFT_SPD_SEP_THRESH,
             LIFT_SPD_D_ALPHA);

    /* 当前位置 = 电机累计圈数 × 编码器分辨率 × 方向符号
     * (编码器在电机本体端, 减速前; LIFT_DIR_SIGN 校正机构方向) */
    float init_pos = Motor_GetTotalRevolutions(MOTOR_LIFT)
                     * (float)LIFT_ENCODER_RESOLUTION
                     * LIFT_DIR_SIGN;

    g_lift.current_position = init_pos;
    /* 初始目标 = 当前位置, 并钳位到软限位内 (上电不动作) */
    g_lift.target_position  = CLAMP_F(init_pos,
                                      (float)LIFT_POSITION_MIN,
                                      (float)LIFT_POSITION_MAX);
    g_lift.target_speed     = 0.0f;
    g_lift.current_speed    = 0.0f;
    g_lift.last_switch_state = 0xFF;
}

/*
 * Lift_FineSpeedCurve: CH2 微调摇杆的"分区速度曲线"
 *
 *   norm: 死区重映射后的归一化偏移 (-1~1, 含符号)
 *   返回: 微调速率系数 (0~1, 符号与输入一致)
 *
 *   微调区间 |norm| ≤ LIFT_FINE_ZONE: 速率 = |norm| × LIFT_FINE_ZONE_GAIN (低速精细)
 *   超出微调区间: 速率从区间端点连续爬升, 满杆 = 1.0 (满速率 LIFT_FINE_STEP_COUNTS)
 *   区间边界连续, 无跳变; 单调递增, 不改变摇杆手感方向
 */
static float Lift_FineSpeedCurve(float norm)
{
    float abs_norm = fabsf(norm);
    float out;

    if (abs_norm <= LIFT_FINE_ZONE) {
        out = abs_norm * LIFT_FINE_ZONE_GAIN;
    } else {
        float at_zone = LIFT_FINE_ZONE * LIFT_FINE_ZONE_GAIN;
        out = at_zone + (abs_norm - LIFT_FINE_ZONE)
                        * (1.0f - at_zone) / (1.0f - LIFT_FINE_ZONE);
    }

    return (norm < 0.0f) ? -out : out;
}

/*
 * 升降控制主循环 (100Hz)
 *   遥控器 ch9 (Se 拨杆) 三档:
 *     拨杆上 (<500)  → 目标 = 最高挡位 (LIFT_POS_UP)
 *     拨杆中 (500~1500) → 锁定当前位置
 *     拨杆下 (>1500) → 目标 = 最低点 (LIFT_POSITION_MIN)
 *
 *   CH2 (left_y) 微调 — 三个档位都生效:
 *     推杆 → 目标位置以固定速率连续积分 (微调), 同时底盘禁止自转
 *     松杆 → 目标停止, 升降保持当前位置
 *
 *   加固点:
 *     1. 目标运行时始终钳位到 [LIFT_POSITION_MIN, MAX] — 软限位, 防顶死机构
 *     2. 拨杆状态变化时清零两环 PID 积分 — 防目标跳变导致输出突跳/饱和
 *     3. 状态机真正工作: MOVING_UP / MOVING_DOWN / HOLD (带滞回防抖)
 *     4. 输出再钳一次到 C620 电流量程 (双保险)
 */
void Lift_Update(void)
{
    g_lift.loop_count++;

    /* 安全检查 */
    if (!rc_data.connected) {
        Lift_EmergencyStop();
        return;
    }

    if (Motor_IsTimeout(MOTOR_LIFT, CAN_MOTOR_TIMEOUT_MS)) {
        Lift_EmergencyStop();
        return;
    }

    /* 1. 更新当前位置 & 当前转速 (位置/速度统一乘 LIFT_DIR_SIGN, 与输出一致) */
    float total_revs = Motor_GetTotalRevolutions(MOTOR_LIFT);
    g_lift.current_position = total_revs * (float)LIFT_ENCODER_RESOLUTION * LIFT_DIR_SIGN;
    g_lift.current_speed    = (float)g_motor.feedback[MOTOR_LIFT].speed_rpm * LIFT_DIR_SIGN;

    /* 2. 读取 ch9 (Se) 拨杆 → 映射到3档 (200~1800, 中位1000) */
    int16_t ch9_val = rc_data.ch9;

    uint8_t sw_idx;
    if (ch9_val < 500) {
        sw_idx = 0;   /* UP → 最高点 */
    } else if (ch9_val > 1500) {
        sw_idx = 2;   /* DOWN → 最低点 */
    } else {
        sw_idx = 1;   /* MID → 微调模式 (锁定 + CH2 微调) */
    }

    /* CH2 (left_y 左摇杆上下) 微调偏移 — 三个拨杆档位都生效 */
    int16_t fine_offset = rc_data.left_y - 1000;   /* -800 ~ +800 */

    /* 死区 + 平滑: 死区内偏移归零; 出死区后减去死区并重新归一化,
     * 移动速率从 0 起连续增大 (满杆仍到满速率), 不在死区边界跳变。
     * 遥控器层已取消内置死区, LIFT_FINE_DEADBAND 即实际有效死区 */
    int32_t off_abs = (fine_offset < 0) ? -(int32_t)fine_offset : (int32_t)fine_offset;
    if (off_abs <= LIFT_FINE_DEADBAND) {
        fine_offset = 0;
    } else {
        float off_sign = (fine_offset > 0) ? 1.0f : -1.0f;
        fine_offset = (int16_t)(off_sign * (off_abs - LIFT_FINE_DEADBAND)
                                * 800.0f / (800.0f - LIFT_FINE_DEADBAND));
    }

    /* 微调标志: CH2 正被推杆使用 → 底盘禁止自转 (供 chassis 查询) */
    g_lift.fine_adjust = (fine_offset != 0) ? 1U : 0U;

    if (sw_idx != g_lift.last_switch_state) {
        g_lift.last_switch_state = sw_idx;

        /* 拨杆状态变化: 目标可能跳变, 清零两环积分防突跳/防积分饱和 */
        PID_Clear(&g_lift.pos_pid);
        PID_Clear(&g_lift.spd_pid);

        if (sw_idx == 1) {
            /* 拨杆回中 → 以当前位置为微调基准 */
            g_lift.target_position = g_lift.current_position;
        } else {
            g_lift.target_position = g_preset_positions[sw_idx];
        }
    }

    /* 2.5 CH2 连续微调目标位置 (任意档位):
     *   摇杆偏移 ±800 → 归一化 → 微调分区曲线 → 每周期(10ms) 移动 ∓(速率×LIFT_FINE_STEP_COUNTS)
     *   微调区间 (LIFT_FINE_ZONE) 内速率更慢, 精细对位; 超出后平滑爬升到满速率。
     *   取负号: 2026-08-05 实车确认微调方向反了, 目标运动方向与摇杆偏移取反。
     *   目标以恒速率积分, 位置环继续工作 → 松杆即停、位置保持, 无模式切换跳变
     *   位置模式 (拨杆上/下) 下同样可微调: 在预设值基础上再被 CH2 推离 */
    if (g_lift.fine_adjust) {
        float fine_norm  = (float)fine_offset / 800.0f;
        float fine_speed = Lift_FineSpeedCurve(fine_norm);
        g_lift.target_position -= fine_speed * LIFT_FINE_STEP_COUNTS;
    }

    /* 3. 软限位钳位 — 目标不允许超出 [MIN, MAX], 防止顶死机构 */
    g_lift.target_position = CLAMP_F(g_lift.target_position,
                                     (float)LIFT_POSITION_MIN,
                                     (float)LIFT_POSITION_MAX);

    /* 4. 外环 — 位置 PID → 速度指令 (rpm) */
    g_lift.target_speed = PID_Calc(&g_lift.pos_pid,
                                   g_lift.target_position,
                                   g_lift.current_position);

    /* 5. 内环 — 速度 PID → 电流指令 */
    float current_out = PID_Calc(&g_lift.spd_pid,
                                 g_lift.target_speed,
                                 g_lift.current_speed);

    /* 双保险: 再钳一次到 C620 电流量程 (PID max_out 已限制, 此处防浮点边界) */
    current_out = CLAMP_F(current_out, (float)C620_CURRENT_MIN, (float)C620_CURRENT_MAX);

    /* 6. 状态机 — 带滞回的到位判定 */
    float pos_err = g_lift.target_position - g_lift.current_position;
    if (g_lift.state == LIFT_STATE_HOLD) {
        /* 已在 HOLD: 误差超过 LIFT_LEAVE_TOLERANCE 才重新判定移动方向 (防抖) */
        if (fabsf(pos_err) > (float)LIFT_LEAVE_TOLERANCE) {
            g_lift.state = (pos_err > 0.0f) ? LIFT_STATE_MOVING_UP
                                            : LIFT_STATE_MOVING_DOWN;
        }
    } else {
        /* 移动中: 误差收敛到 LIFT_ARRIVED_TOLERANCE 以内 → 到位保持 */
        if (fabsf(pos_err) <= (float)LIFT_ARRIVED_TOLERANCE) {
            g_lift.state = LIFT_STATE_HOLD;
        } else {
            g_lift.state = (pos_err > 0.0f) ? LIFT_STATE_MOVING_UP
                                            : LIFT_STATE_MOVING_DOWN;
        }
    }

    /* 7. 输出 — 电流方向修正 (与 LIFT_DIR_SIGN 保持一致):
     *    正输出 → 平台上升 → 计数增大 → 位置收敛。
     *    只取反电流而不取反反馈会把负反馈变正反馈 → 上电疯转, 三者必须一起取反 */
    g_motor.current_setpoint[MOTOR_LIFT] = (int16_t)(current_out * LIFT_DIR_SIGN);
    Motor_SendLiftCurrent();
}

void Lift_EmergencyStop(void)
{
    g_lift.state = LIFT_STATE_ESTOP;
    g_lift.target_speed = 0.0f;
    g_motor.current_setpoint[MOTOR_LIFT] = 0;
    PID_Clear(&g_lift.pos_pid);
    PID_Clear(&g_lift.spd_pid);
    Motor_SendLiftCurrent();
}

Lift_State_t Lift_GetState(void)
{
    return g_lift.state;
}

uint8_t Lift_IsFineAdjust(void)
{
    return g_lift.fine_adjust;
}

/*
 * 编码器计数值 → 平台高度 (mm)
 *
 * 同步带传动链:
 *   编码器计数 → 电机转数 → 减速后输出转数 → 同步带轮线位移(平台高度)
 *
 *   平台高度(mm) = counts / 8192 / LIFT_REDUCTION_RATIO × LIFT_SCREW_LEAD_MM
 *                  └──电机转数──┘   └─────输出转数─────┘ └─带轮每转位移(mm)─┘
 *
 *   LIFT_SCREW_LEAD_MM: 原意为"丝杆导程", 同步带方案中表示带轮每转的线位移
 *                        = π × 带轮节圆直径 = 带轮齿数 × 同步带节距
 */
float Lift_GetHeightMM(void)
{
    float counts = g_lift.current_position;
    return counts / (float)LIFT_ENCODER_RESOLUTION
           / LIFT_REDUCTION_RATIO * LIFT_SCREW_LEAD_MM;
}
