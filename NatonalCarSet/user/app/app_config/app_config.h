/**
 * @file    app_config.h
 * @brief   系统全局配置文件 — 所有可调参数集中管理
 *
 * @note    本文件是整车的"参数单页"，涵盖:
 *          - 系统控制频率
 *          - CAN 通信协议 (M3508/C620)
 *          - 底盘机械尺寸 + 麦科勒姆轮参数
 *          - 升降机构传动参数
 *          - PID 控制器默认参数
 *          - GPIO 引脚分配
 *          - FreeRTOS 任务优先级与栈大小
 *          - 通用工具宏
 *
 *          修改任一参数后重新编译即可生效，无需改动业务代码。
 *          所有带 TODO 标记的参数需在实车上标定。
 ******************************************************************************
 */

#ifndef __APP_CONFIG_H__
#define __APP_CONFIG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <string.h>

/* ========================================================================= */
/*  1. 系统控制频率                                                            */
/*                                                                            */
/*  频率选择原则:                                                              */
/*  - 底盘 200Hz: M3508/C620 响应带宽 ~100-200Hz, 再高无意义                  */
/*  - 升降 100Hz: 位置环比速度环慢, 且丝杆机构惯性大                          */
/*  - 监控  10Hz: 100ms 对超时检测足够快 (CAN_TIMEOUT=200ms -> 2周期触发)     */
/* ========================================================================= */

/** @brief 底盘控制频率 (Hz) — 每 5ms 执行一次 Chassis_Update() */
#define CHASSIS_CONTROL_FREQ_HZ        200U

/** @brief 升降控制频率 (Hz) — 每 10ms 执行一次 Lift_Update() */
#define LIFT_CONTROL_FREQ_HZ           100U

/** @brief 安全监控频率 (Hz) — 每 100ms 执行一次 Watchdog_Update() */
#define WATCHDOG_FREQ_HZ               10U

/*
 * 控制周期 (ms) — 由频率自动推算, 供 freertos.c 中 vTaskDelayUntil 使用
 * 例: CHASSIS_PERIOD_MS = 1000/200 = 5ms
 */
#define CHASSIS_PERIOD_MS              (1000U / CHASSIS_CONTROL_FREQ_HZ)
#define LIFT_PERIOD_MS                 (1000U / LIFT_CONTROL_FREQ_HZ)

/* ========================================================================= */
/*  2. CAN 通信参数 (M3508 电机 + C620 电调)                                   */
/*                                                                            */
/*  CAN 波特率计算:                                                            */
/*    BaudRate = APB1_CLK / Prescaler / (1 + BS1 + BS2)                       */
/*            = 42MHz / 3 / (1 + 10 + 3)                                      */
/*            = 42MHz / 3 / 14 = 1 Mbps ✓                                     */
/*                                                                            */
/*  C620 电调控制协议:                                                         */
/*  - 电流指令: int16_t, ±16384 对应 ±20A (实测线性度在此范围内良好)           */
/*  - 底盘 4 电机: CAN ID 0x200, DLC=8, 4×int16_t 大端序                      */
/*  - 升降 1 电机: CAN ID 0x1FF, DLC=8, 1×int16_t 大端序 (后 6 字节填 0)      */
/*  - 反馈帧:     CAN ID 0x201~0x205, 每帧 8 bytes:                           */
/*                [0-1] 机械角度 0-8191 (uint16_t 大端)                        */
/*                [2-3] 转速 rpm (int16_t 大端)                                */
/*                [4-5] 实际力矩电流 (int16_t 大端)                             */
/*                [6]   温度 ℃                                                */
/*                [7]   保留                                                   */
/* ========================================================================= */

/** @brief CAN 预分频器 — APB1_CLK(42MHz) ÷ 3 = 14MHz 进入 CAN 模块 */
#define CAN_PRESCALER                  3U

/** @brief Bit Segment 1 — 10 个时间量子 (采样点 ~78%) */
#define CAN_BS1                        10U

/** @brief Bit Segment 2 — 3 个时间量子 */
#define CAN_BS2                        3U

/** @brief Synchronization Jump Width — 1 个时间量子 (标准值) */
#define CAN_SJW                        1U

/** @brief 底盘电机 CAN 发送 ID — 数据帧 0x200 (标准帧, 11-bit) */
#define CAN_TX_ID_CHASSIS              0x200U

/** @brief 升降电机 CAN 发送 ID — 数据帧 0x1FF (标准帧, 11-bit) */
#define CAN_TX_ID_LIFT                 0x1FFU

/** @brief 电机反馈 ID 起始值 — 0x201 对应 电机1, 0x205 对应 电机5 */
#define CAN_RX_ID_BASE                 0x201U

/*
 * C620 电流指令范围
 * 16384 = 0x4000, 对应电调的最大正向力矩电流 (约 20A)
 * 负值对应反向力矩
 * 实际输出会被 PID 输出限幅钳位到此范围内
 */
#define C620_CURRENT_MAX               16384
#define C620_CURRENT_MIN              (-16384)

/*
 * CAN 电机通信超时 (ms)
 *
 * M3508 反馈帧以 1kHz 发送 (每 1ms 一帧),
 * 200ms 超时意味着连续丢失 200 帧才会触发保护。
 * 设置依据:
 *  - 太短 (< 50ms): CAN 总线偶然拥堵可能误触发
 *  - 太长 (> 500ms): 电机已失控半秒才发现, 太危险
 *  - 200ms:   2 个 watchdog 周期 (100ms×2) 内触发, 响应及时
 */
#define CAN_MOTOR_TIMEOUT_MS           200U

/* ========================================================================= */
/*  3. 底盘机械参数 (麦科勒姆轮)                                               */
/*                                                                            */
/*  轮子布局 (俯视图, ↑前进方向):                                              */
/*                                                                            */
/*           前方                                                              */
/*       ┌──────────┐                                                        */
/*       │  FL   FR │  ← FL=Motor[1]  FR=Motor[0]                            */
/*       │  2    1  │                                                        */
/*       │          │                                                        */
/*       │  3    4  │  ← RL=Motor[2]  RR=Motor[3]                            */
/*       │  RL   RR │                                                        */
/*       └──────────┘                                                        */
/*                                                                            */
/*  运动学参数:                                                                */
/*    K = Lx + Ly  (Lx=半轮距, Ly=半轴距)                                      */
/*    V_FR =  Vx - Vy - K*Wz                                                 */
/*    V_FL =  Vx + Vy + K*Wz                                                 */
/*    V_RL = -Vx + Vy - K*Wz                                                 */
/*    V_RR = -Vx - Vy + K*Wz                                                 */
/*                                                                            */
/*  ⚠ 以下尺寸参数必须根据实际车辆测量后填入, 当前为占位默认值                  */
/* ========================================================================= */

/*
 * CHASSIS_LX: 半轮距 (左右轮中心距的一半, 单位 mm)
 *
 * 测量方法: 从机器人正上方测量左右轮触地点中心之间的距离 ÷ 2
 * 当前默认 150mm → 整车宽约 300mm (小车尺寸)
 * TODO: 实车测量
 */
#define CHASSIS_LX                     242.0f

/*
 * CHASSIS_LY: 半轴距 (前后轮中心距的一半, 单位 mm)
 *
 * 测量方法: 从机器人侧面测量前后轮触地点中心之间的距离 ÷ 2
 * 当前默认 150mm → 整车长约 300mm
 * TODO: 实车测量
 */
#define CHASSIS_LY                     190.0f

/*
 * WHEEL_RADIUS_MM: 麦科勒姆轮子半径 (mm)
 *
 * 测量方法: 用卡尺测量轮毂外径 ÷ 2
 * 当前默认 50mm → 直径 100mm (常见竞赛轮尺寸)
 * TODO: 实车测量
 * 注意: 橡胶形变会导致有效半径略小于几何半径, 可标定时微调
 */
#define WHEEL_RADIUS_MM                22.5f

/*
 * 线速度 (mm/s) → 轮转速 (rpm) 换算系数 (物理值, 不改)
 *
 * 推导:
 *   轮周长 = 2 × π × WHEEL_RADIUS_MM (mm)
 *   轮转速 = 线速度 / 轮周长 × 60 (rpm)
 */
#define WHEEL_MMPS_TO_RPM              (60.0f / (2.0f * 3.14159265358979f * WHEEL_RADIUS_MM))

/*
 * CHASSIS_MAX_RPM: 满摇杆时电机目标转速 (rpm, 电机本体端)
 *
 *   ★ 这是唯一决定满杆动力输出的参数, 直接改它即可调节最大车速。
 *   M3508 空载最高约 9000 rpm (本体), 建议初调设 3000~5000, 逐步放开。
 *   TODO: 实车标定
 */
#define CHASSIS_MAX_RPM                6500.0f

/*
 * CHASSIS_MAX_LINEAR_SPEED_MMPS: 底盘满摇杆"标称"线速度 (mm/s)
 *
 *   仅作为 UI/日志中的参考值, 不影响实际动力输出。
 *   实际最大轮转速由上面的 CHASSIS_MAX_RPM 决定。
 *
 *   缩小此值时 SCALE 自动补偿, 满杆转速恒定为 CHASSIS_MAX_RPM。
 *   例: MAX_LINEAR=6000, MAX_RPM=5729 → 满杆时显示 6000mm/s 但轮转 5729rpm。
 */
#define CHASSIS_MAX_LINEAR_SPEED_MMPS   300.0f

/*
 * CHASSIS_STRAFE_SCALE: 左右平移 (Vy, 右杆 CH1) 速度系数
 *
 * 仅作用于横移, 前后 (Vx) 不受影响。 <1 减速, >1 加速。
 */
#define CHASSIS_STRAFE_SCALE             0.8f

/*
 * CHASSIS_ACCEL_LIMIT_MMPS2 / CHASSIS_DECEL_LIMIT_MMPS2:
 *   速度指令限速 (平滑加减速, 单位 mm/s²)
 *
 *   Chassis_Update 每 5ms 一个周期, 目标速度每周期最多变化
 *   limit × 0.005 (mm/s)。松杆回中时目标按减速斜率线性降到 0,
 *   不再瞬间归零刹死, 车会平稳滑行停下。
 *
 *   调法:
 *     松杆还是顿挫/刹得太快 → 减小 CHASSIS_DECEL_LIMIT_MMPS2
 *     松杆后溜太远停不下来 → 增大 CHASSIS_DECEL_LIMIT_MMPS2
 *     起步反应慢/加速无力   → 增大 CHASSIS_ACCEL_LIMIT_MMPS2
 *
 *   当前默认: 满杆 300mm/s, 加速 ~0.1s 到顶, 松杆 ~0.5s 停稳
 */
#define CHASSIS_ACCEL_LIMIT_MMPS2        3000.0f
#define CHASSIS_DECEL_LIMIT_MMPS2         600.0f

/*
 * CHASSIS_ANG_ACCEL_LIMIT_RADPS2 / CHASSIS_ANG_DECEL_LIMIT_RADPS2:
 *   自转角速度 (Wz) 的平滑限速 (单位 rad/s²), 同理作用于旋转启停。
 *   满杆 Wz ≈ 0.698 rad/s (40°/s), DECEL=3.14 → 0.22s 转停。
 */
#define CHASSIS_ANG_ACCEL_LIMIT_RADPS2    6.28f
#define CHASSIS_ANG_DECEL_LIMIT_RADPS2    3.14f

/*
 * CHASSIS_RPM_SCALE: 自动补偿因子
 *
 *   保证: 满摇杆 rpm = CHASSIS_MAX_RPM, 不受 MAX_LINEAR_SPEED 变化影响。
 */
#define CHASSIS_RPM_SCALE              (CHASSIS_MAX_RPM / (CHASSIS_MAX_LINEAR_SPEED_MMPS * WHEEL_MMPS_TO_RPM))

/*
 * CHASSIS_MAX_ANGULAR_SPEED_RADPS: 底盘最大自转角速度 (rad/s)
 *
 * 摇杆推到底时, 底盘的自转速度。
 * 0.698132 rad/s = 0.1111 转/秒 (40°/s)
 * 已调历程: 6.28318 (360°/s) → 3.14159 (180°/s) → 0.785398 (45°/s) → 0.392699 (22.5°/s)
 *           → 0.589048 (33.75°/s) → 0.698132 (40°/s)
 */
#define CHASSIS_MAX_ANGULAR_SPEED_RADPS (40.0f * 3.14159265358979f / 180.0f)

/*
 * CHASSIS_LINEAR_DEADBAND: 前进/平移通道 (Vx/Vy, 右杆 CH3/CH1) 归一化死区 (0~1)
 *
 * 前进 (Vx) 与横移 (Vy) 共用, 固定小死区 (不做平滑重映射)。
 * 注意: 遥控器内置 JOYSTICK_DEADBAND=50 (归一化≈0.083) 已保证回中零位,
 *       此宏须大于 0.083 才会真正扩大有效死区。
 * 换算: 0.10 → 摇杆偏移约 60 (200~1800 刻度), 约 7.5% 行程。
 */
#define CHASSIS_LINEAR_DEADBAND          0.20f

/*
 * CHASSIS_ROT_DEADBAND: 旋转通道 (CH4 / left_x) 归一化死区 (0~1)
 *
 * 误触旋转主要由升降微调占用左摇杆导致 (推 CH2 调升降时对角抖动带出 CH4),
 * 已通过加大 LIFT_FINE_DEADBAND + 微调期间强制 Wz=0 抑制, 故旋转死区可减小。
 *
 * 换算: 0.05 → 摇杆偏移约 30 (200~1800 刻度), 约 3.75% 行程;
 * 低于遥控器内置 JOYSTICK_DEADBAND=50 时, 有效死区由遥控器 50 决定。
 * 死区边界做平滑: 出死区后输出从 0 起线性增大 (见 chassis.c)。
 */
#define CHASSIS_ROT_DEADBAND             0.05f

/*
 * RC_JOYSTICK_SCALE: 摇杆校准系数
 *
 *   AT9S 遥控器由于 EPA 限位或机械行程, 满杆时 ch 值可能达不到理论 1800。
 *   此时 VOFA 波形显示满杆不到 1.0, 通过此系数补偿。
 *
 *   校准方法:
 *     1. 注释掉此行 (=1.0), 编译烧录, 推满前进看 VOFA 通道1 的峰值
 *     2. SCALE = 1.0 / 峰值  (如峰值 0.75 → SCALE = 1.333)
 *     3. 重新编译烧录, 满杆应到 1.0
 *
 *   或者直接在 AT9S 遥控器菜单 End Point Adjust 中将各通道设为 100%。
 */
#define RC_JOYSTICK_SCALE              1.333f   /* 1.0/0.75, AT9S 满杆实测峰值 */

/*
 * M3508_REDUCTION_RATIO: M3508 减速箱减速比
 *
 * M3508 电机本体是 19.2032:1 行星减速箱
 *  电机输出轴 1 圈 = 编码器 8192 计数 (注: 编码器在电机本体端, 非输出轴)
 *  电机转子 19.2032 圈 = 输出轴 1 圈
 *
 *  但在本项目速度环中, 我们控制的是电机本体转速 (g_motor.feedback[].speed_rpm),
 *  这个值已经是电机本体 rpm, 所以运动学解算时不需要除以减速比。
 *  此宏保留供后续需要输出轴转速/位置的计算场景使用。
 */
#define M3508_REDUCTION_RATIO          19.2032f

/** @brief 底盘电机数量 — 4 个麦科勒姆轮 (不含升降电机) */
#define CHASSIS_MOTOR_COUNT            4U

/* ========================================================================= */
/*  4. 升降机构参数 (同步带传动)                                               */
/*                                                                            */
/*  传动链:                                                                    */
/*    电机(M3508) → 行星减速箱(19:1) → 主动同步带轮 → 同步带 → 从动同步带轮    */
/*                                       (motor side)           (load side)   */
/*                                                                            */
/*  位置换算 (电机输出轴 → 平台位移):                                          */
/*    平台高度(mm) = 编码器计数值 / 8192 / 减速比 × (π × 同步带轮节圆直径)     */
/*                                                                            */
/*  或等效为:                                                                  */
/*    平台高度(mm) = 编码器计数值 / 8192 / 减速比 × 同步带轮齿数 × 节距        */
/*                  └──────────┬──────────┘   └──────┬──────┘                 */
/*                   电机转数 (rev)              同步带轮周长 (mm)               */
/*                                                                            */
/*  ⚠ 以下参数必须根据实际机械结构测量后填入                                    */
/* ========================================================================= */

/*
 * LIFT_REDUCTION_RATIO: 升降机构总减速比
 *
 * 总减速比 = M3508 行星减速箱 × 同步带轮减速比
 *   M3508 自带: 19.2032:1 (固定)
 *   同步带轮减速比 = 从动轮齿数 / 主动轮齿数
 *
 * 例: 主动轮 20T, 从动轮 40T → 带轮减速比 2:1
 *     总减速比 = 19.2032 × 2 = 38.4064:1
 *
 * 如果主动轮和从动轮齿数相同 (1:1): 总减速比 = 19.2032 (仅行星减速)
 * TODO: 数两个同步带轮的齿数, 确认实际减速比
 */
#define LIFT_REDUCTION_RATIO           19.2032f

/*
 * LIFT_SCREW_LEAD_MM: 同步带传动 — 带轮每转的平台位移 (mm/rev)
 *
 * ★ 迭代标定记录 (2026-08-05), 依据最高挡位尺测:
 *   189.68 → 194.85 (477mm 显示 ≈ 490mm 实测)
 *   → 198.91  (480mm 挡位 387529 counts 尺测 490mm, 换算偏低 ~2% → 修正导程)
 *   LIFT_SCREW_LEAD_MM = 194.85 × 490/480 ≈ 198.91 mm/rev
 *
 * 机构硬限位 = 大电机 3 圈 (≈ 596.73mm), 与软限位 LIFT_POSITION_MAX_MM 相同:
 *   软限位即按硬限位设定, 目标不允许超出, 防止顶死机构。
 * 若机构改动需重新标定, 改此值即可, LIFT_COUNTS_PER_MM 自动跟随。
 */
#define LIFT_SCREW_LEAD_MM             198.91f

/*
 * LIFT_ENCODER_RESOLUTION: M3508 编码器分辨率
 *
 * M3508 使用 14-bit 磁编码器 (实际有效 13-bit = 8192 counts/rev)
 * 编码器安装在电机本体端 (减速前)
 * 8192 = 2^13, 一圈 8192 个脉冲
 */
#define LIFT_ENCODER_RESOLUTION        8192U

/*
 * 升降位置单位换算 (mm → 编码器计数)
 *
 *   counts = 高度(mm) × 8192 × 19.2032 / LIFT_SCREW_LEAD_MM
 *          = 高度(mm) × LIFT_COUNTS_PER_MM
 *   LIFT_COUNTS_PER_MM ≈ 790.9 (由实测 480mm 挡位 ≈ 尺测490mm 迭代标定)
 *
 * 下方所有位置宏 (软限位 / 预设挡位) 一律以 mm 为单位填写,
 * 用 LIFT_MM_TO_COUNTS 自动换算成编码器计数。
 */
#define LIFT_COUNTS_PER_MM             ((float)LIFT_ENCODER_RESOLUTION * LIFT_REDUCTION_RATIO / LIFT_SCREW_LEAD_MM)
#define LIFT_MM_TO_COUNTS(mm)          ((int32_t)((mm) * LIFT_COUNTS_PER_MM + 0.5f))

/*
 * LIFT_DIR_SIGN: 升降电机方向符号
 *
 * 本车实测: 正电流指令 → 编码器计数增大, 但机构平台"下降"
 * (编码器计数方向与平台运动方向相反)。
 * 若只取反电流指令、反馈仍按原方向, 位置环会变成正反馈 → 上电即疯转。
 * 因此位置/速度反馈与电流指令三者必须统一乘以该符号 (=-1):
 *   计数增大 = 平台上升, 正输出 = 上升。
 * 若日后改动机构/接线使方向恢复正常, 改回 +1.0f 即可。
 */
#define LIFT_DIR_SIGN                   (-1.0f)

/* 升降机构软限位 — 单位: mm (经 LIFT_MM_TO_COUNTS 自动换算为编码器计数) */
#define LIFT_POSITION_MIN_MM          0.0f      /**< 最低点 */
#define LIFT_POSITION_MAX_MM          635.0f   /**< 最高软限位 ≈ 大电机 3 圈 (3 × LIFT_SCREW_LEAD_MM) */
#define LIFT_POSITION_MIN             LIFT_MM_TO_COUNTS(LIFT_POSITION_MIN_MM)
#define LIFT_POSITION_MAX             LIFT_MM_TO_COUNTS(LIFT_POSITION_MAX_MM)

/*
 * 预设挡位 — 单位: mm (拨杆三档目标, 自动换算为编码器计数)
 //此处数据失真自己微调
 * 拨杆下 → LIFT_POS_DOWN  = 最低点 (LIFT_POSITION_MIN)
 *
 * 最高挡位 < 软限位上限 (596.73mm ≈ 大电机 3 圈):
 *   挡位 480mm 距软限位还有 ~117mm 余量, CH2 微调可上探而不顶硬限位。
 */
#define LIFT_POS_UP_MM                502.0f
#define LIFT_POS_UP                   LIFT_MM_TO_COUNTS(LIFT_POS_UP_MM)
#define LIFT_POS_DOWN                 LIFT_POSITION_MIN

/*
 * 微调模式参数 (CH9 拨杆回中时启用)
 *
 * 控制方式:
 *   CH9(Se) 回中 → 进入微调模式, CH2(left_y 左摇杆上下) 连续微调升降位置
 *   摇杆偏移 ±800 → 目标位置每周期(10ms) 移动 ±LIFT_FINE_STEP_COUNTS 计数
 *
 *   LIFT_FINE_STEP_COUNTS 默认 400 counts/周期 = 40000 counts/s
 *     → 全程 ~472000 counts 约 12s 走完 (精细调速, 慢推慢移)
 *   - 需要更快 → 加大此值; 需要更精细 → 减小此值
 *
 *   LIFT_FINE_DEADBAND: 摇杆中位死区 (映射后 200~1800, 中位1000)
 *   需大于遥控器内置 JOYSTICK_DEADBAND=50 才真正生效 (合计有效死区 = 此值)。
 *   死区边界做平滑: 出死区后移动速率从 0 起线性增大 (见 lift.c)
 */
#define LIFT_FINE_STEP_COUNTS          600
#define LIFT_FINE_DEADBAND             200

/* ========================================================================= */
/*  5. PID 控制器默认参数                                                      */
/*                                                                            */
/*  每个参数的作用 (以 PID_TypeDef 中的 PID_Init 参数顺序排列):                 */
/*    PID_Init(pid, Kp, Ki, Kd, max_out, max_i, integral_sep_thresh, d_alpha) */
/*                                                                            */
/*    Kp                  : 比例增益 — 误差直接放大, 决定响应快慢                */
/*    Ki                  : 积分增益 — 消除稳态误差, 过大→积分饱和→超调         */
/*    Kd                  : 微分增益 — 抑制振荡, 预测误差趋势                    */
/*    max_out             : 输出限幅 — 最终输出钳位到 ±max_out                   */
/*    max_i               : 积分限幅 — 防止积分项无限累积 (积分饱和)              */
/*    integral_sep_thresh : 积分分离阈值 — |误差|>此值时清零积分, 防止大偏差时    */
/*                          积分累积导致超调                                    */
/*    d_alpha             : 微分滤波系数 (0~1) — 越小滤波越强                    */
/*                          d_filtered = α×d_err + (1-α)×d_filtered             */
/*                                                                            */
/*  ⚠ 所有 PID 参数需在实车上整定, 以下为保守默认值                             */
/* ========================================================================= */

/* ---------------------------------------------------------------------------*/
/*  5.1 底盘速度环 PID (4 轮独立, 各一个 PID 控制器)                            */
/*                                                                            */
/*  被控对象: M3508 电机本体转速 (rpm)                                          */
/*  执行器:   C620 电调电流指令 (±16384)                                       */
/*  控制频率: 200Hz                                                            */
/* ---------------------------------------------------------------------------*/

/** @brief 速度环比例增益 — 提供即时响应, 太高会振荡 */
#define CHASSIS_SPEED_KP               8.0f

/** @brief 速度环积分增益 — 累积锁止力, 消除静差 */
#define CHASSIS_SPEED_KI               1.5f

/** @brief 速度环微分增益 — 阻尼振荡, 预测转速变化趋势 */
#define CHASSIS_SPEED_KD               0.3f

/** @brief 输出限幅 — ±16384 = C620 全电流范围 */
#define CHASSIS_SPEED_MAX_OUT          16384.0f

/** @brief 积分限幅 — 锁止时约 30% 最大电流即可 */
#define CHASSIS_SPEED_MAX_I            5000.0f

/** @brief 积分分离阈值 — |速度误差| > 500rpm 时清零积分 (急加速/急减速阶段) */
#define CHASSIS_SPEED_SEP_THRESH       500.0f

/*
 * 微分滤波系数 — d_alpha = 0.1
 * 含义: 新微分值权重 10%, 历史滤波值权重 90%
 *       d_filtered = 0.1×d_err + 0.9×d_filtered
 * 越接近 0 → 滤波越强 → 微分输出越平滑 → 但相位滞后越大
 * 越接近 1 → 滤波越弱 → 微分输出越接近原始 → 但噪声放大明显
 * 0.1 适合电机速度环 (转速信号有一定的传感器噪声)
 */
#define CHASSIS_SPEED_D_ALPHA          0.1f

/* ---------------------------------------------------------------------------*/
/*  5.2 升降串级 PID (位置环 → 速度环 → 电流)                                  */
/*                                                                            */
/*  控制架构:                                                                   */
/*    外环 — 位置PID: 位置误差(counts) → 速度指令(rpm)                           */
/*    内环 — 速度PID: 速度误差(rpm)    → 电流指令(±16384)                        */
/*                                                                            */
/*  调参顺序: 先调速度环 (电机能稳速转动), 再调位置环 (平台能准确定位)           */
/*  控制频率: 100Hz (内外环同频)                                                */
/* ---------------------------------------------------------------------------*/

/* ---- 5.2.1 位置环 (外环, 调速度环时旁路) ---- */

/*
 * 调速度环用 — 固定速度指令 (rpm, 电机本体端)
 * 拨杆向上 = +LIFT_TUNE_SPEED_RPM (正转)
 * 拨杆向下 = -LIFT_TUNE_SPEED_RPM (反转)
 * 调好速度环后加回外环, 此宏即可废弃
 *
 * 建议从 1000 rpm 起, 逐步加大到 3000~5000
 */
#define LIFT_TUNE_SPEED_RPM            1000.0f

/*
 * 位置环输出 = 速度指令 (rpm, 电机本体端)
 *
 * 例: 位置误差 10000 counts × Kp 0.3 = 3000 rpm
 *     位置误差   100 counts × Kp 0.3 =   30 rpm (接近目标时蠕行)
 */
#define LIFT_POS_KP                    0.10f

/** @brief 位置环积分增益 — 消除自重及摩擦导致的稳态位置误差 */
#define LIFT_POS_KI                    0.05f

/** @brief 位置环微分增益 — 预判位置变化趋势, 抑制到位过冲 */
#define LIFT_POS_KD                    0.02f

/*
 * 输出限幅 — ±3000 rpm (电机本体端)
 * 升降机构通过减速比降速, 3000rpm 本体 ≈ 156rpm 输出轴, 足够快
 */
#define LIFT_POS_MAX_OUT               3000.0f

/** @brief 位置环积分限幅 — ±1000 rpm (积分项贡献的速度上限) */
#define LIFT_POS_MAX_I                 1000.0f

/** @brief 位置环积分分离阈值 — |位置误差| > 5000 counts 时清零积分 */
#define LIFT_POS_SEP_THRESH            5000.0f

/** @brief 位置环微分滤波系数 — 0.05, 位置信号噪声大, 加强滤波 */
#define LIFT_POS_D_ALPHA               0.05f

/* ---- 5.2.2 速度环 (内环, 先调) ---- */

/*
 * 速度环 Kp = 8.0
 * 例: 速度误差 500rpm × 8.0 = 4000 电流, 响应快但不振荡
 */
#define LIFT_SPD_KP                    6.5f

/** @brief 速度环积分增益 — 消除稳态转速误差 (如负载变化) */
#define LIFT_SPD_KI                    0.05f

/** @brief 速度环微分增益 — 抑制转速突变, 对负载突变起阻尼作用 */
#define LIFT_SPD_KD                    0.1f

/** @brief 速度环输出限幅 — ±16384 (C620 全量程) */
#define LIFT_SPD_MAX_OUT               16384.0f

/** @brief 速度环积分限幅 — ±5000, 锁止力约占 30% 最大电流 */
#define LIFT_SPD_MAX_I                 5000.0f

/** @brief 速度环积分分离阈值 — |速度误差| > 500rpm 时清零积分 */
#define LIFT_SPD_SEP_THRESH            500.0f

/** @brief 速度环微分滤波系数 — 0.1, 转速信号噪声中等 */
#define LIFT_SPD_D_ALPHA               0.1f

/* ========================================================================= */
/*  6. 电磁阀 GPIO                                                             */
/*                                                                            */
/*  4 路电磁阀 (PB12~PB15), 各自由遥控器辅助通道独立控制:                       */
/*    PB12 → CH5 (Sa)    PB13 → CH7 (Sc)                                      */
/*    PB14 → CH8 (Sd)    PB15 → CH6 (Sb)                                      */
/*                                                                            */
/*  - 高电平 (SET)   = 电磁阀通电 → 气路打开                                     */
/*  - 低电平 (RESET) = 电磁阀断电 → 气路关闭 (默认安全状态)                       */
/*                                                                            */
/*  通道占用说明:                                                              */
/*    CH10 = 陀螺仪航向保持开关, CH9 = 升降, 故阀只用 5/6/7/8                  */
/*                                                                            */
/*  注意: PB12~15 直接驱动需要确认 STM32 的 3.3V IO 能否驱动电磁阀              */
/*        如果是 12V/24V 电磁阀, 必须加 MOS 管或继电器驱动电路!!!               */
/* ========================================================================= */

/** @brief 电磁阀控制端口 — GPIOB */
#define VALVE_GPIO_PORT                GPIOB

/** @brief 电磁阀 1 — PB12, 遥控器 CH5 (Sa) */
#define VALVE_1_PIN                    GPIO_PIN_12
#define VALVE_1_RC_CHANNEL             5U

/** @brief 电磁阀 2 — PB13, 遥控器 CH7 (Sc) */
#define VALVE_2_PIN                    GPIO_PIN_13
#define VALVE_2_RC_CHANNEL             7U

/** @brief 电磁阀 3 — PB14, 遥控器 CH8 (Sd) */
#define VALVE_3_PIN                    GPIO_PIN_14
#define VALVE_3_RC_CHANNEL             8U

/** @brief 电磁阀 4 — PB15, 遥控器 CH6 (Sb) */
#define VALVE_4_PIN                    GPIO_PIN_15
#define VALVE_4_RC_CHANNEL             6U

/** @brief 电磁阀数量 — 数组大小/循环上界 */
#define VALVE_COUNT                    4U

/* ========================================================================= */
/*  7. LED 状态指示                                                             */
/*                                                                            */
/*  PE1 驱动状态 LED:                                                          */
/*  - 常亮:          系统 OK                                                   */
/*  - 500ms 周期闪烁: 遥控器信号丢失                                            */
/*  - 200ms 周期闪烁: 电机 CAN 通信超时                                         */
/*  - 灭:            急停 (ESTOP)                                              */
/* ========================================================================= */

/** @brief LED 控制端口 — GPIOE */
#define LED_GPIO_PORT                  GPIOE

/** @brief LED 控制引脚 — PE1 (物理引脚) */
#define LED_GPIO_PIN                   GPIO_PIN_1

/* ========================================================================= */
/*  8. 电机索引 (0-based)                                                      */
/*                                                                            */
/*  对应关系:                                                                  */
/*    g_motor.feedback[idx]      ← CAN ID 0x201 + idx                         */
/*    g_motor.current_setpoint[idx] → 打包进 CAN 帧发送                        */
/*                                                                            */
/*  索引 0-3: 底盘麦克纳姆轮 (对应 CAN ID 0x201~0x204)                          */
/*  索引 4:   升降机构电机   (对应 CAN ID 0x205)                                */
/* ========================================================================= */

/** @brief 电机总数 — 4 底盘 + 1 升降 */
#define MOTOR_COUNT                    5U

#define MOTOR_CHASSIS_1                0U   /**< 右前轮 (FR) — CAN ID 0x201 */
#define MOTOR_CHASSIS_2                1U   /**< 左前轮 (FL) — CAN ID 0x202 */
#define MOTOR_CHASSIS_3                2U   /**< 左后轮 (RL) — CAN ID 0x203 */
#define MOTOR_CHASSIS_4                3U   /**< 右后轮 (RR) — CAN ID 0x204 */
#define MOTOR_LIFT                     4U   /**< 升降电机   — CAN ID 0x205 */

/* ========================================================================= */
/*  9. FreeRTOS 任务优先级与栈大小                                              */
/*                                                                            */
/*  优先级从高到低:                                                            */
/*    watchdog (High)                                                   最高   */
/*      └─ 必须能在任何情况下抢占 CPU 执行急停                                   */
/*    remote (AboveNormal)                                                    */
/*      └─ 后续如需加入滤波, 高于控制任务可保证数据及时处理                        */
/*    chassis / lift / valve (Normal) — 同级, 时间片轮转                       */
/*      └─ 控制任务业务量相当, 无需区分优先级                                     */
/*      └─ 阀门原为 BelowNormal(最低): WS2812 任务(同 Normal)存在忙等时          */
/*         (osDelay(0) 轮询 DMA), 会持续占住 CPU 使更低优先级永远得不到调度,     */
/*         故阀门提升到 Normal, 与底盘/升降一起时间片轮转, 保证电磁阀正常执行     */
/*                                                                            */
/*  CMSIS_V2 优先级枚举对照 (数值越大优先级越高):                                */
/*    osPriorityLow        = 8   (仅 Idle 使用)                                */
/*    osPriorityBelowNormal = 16                                               */
/*    osPriorityNormal      = 24                                               */
/*    osPriorityAboveNormal = 32                                               */
/*    osPriorityHigh        = 40                                               */
/*    osPriorityRealtime    = 56                                               */
/*                                                                            */
/*  栈大小说明:                                                                */
/*    CMSIS-RTOS2 的 osThreadAttr_t.stack_size 单位是 byte。                    */
/*    栈溢出 → MemManage Fault → HardFault, 排查困难, 务必留足余量               */
/* ========================================================================= */

/* --- 任务优先级 --- */

#define PRIO_WATCHDOG                  osPriorityHigh          /**< 安全监控 — 最高 (40) */
#define PRIO_REMOTE                    osPriorityAboveNormal   /**< 遥控器   — 次高 (32) */
#define PRIO_CHASSIS                   osPriorityNormal        /**< 底盘控制 — 正常 (24) */
#define PRIO_LIFT                      osPriorityNormal        /**< 升降控制 — 正常 (24), 与底盘同级 */
#define PRIO_VALVE                     osPriorityNormal        /**< 电磁阀   — 与底盘同级 (24), 防忙等饿死 */

/* --- 任务栈大小 (单位: bytes) --- */

#define STACK_WATCHDOG                1024U  /**< 1 KiB — 简单状态检查 + LED 控制 */
#define STACK_REMOTE                  1024U  /**< 1 KiB — 遥控超时检查 */
#define STACK_CHASSIS                 2048U  /**< 2 KiB — 浮点运算 + 4路PID */
#define STACK_LIFT                    2048U  /**< 2 KiB — 浮点运算 + 串级PID */
#define STACK_VALVE                    512U  /**< 512 bytes — GPIO 读写 */

/* ========================================================================= */
/*  10. 通用工具宏                                                             */
/* ========================================================================= */

/*
 * ARRAY_SIZE: 编译期计算数组元素个数
 *
 * 例: int arr[10];
 *     ARRAY_SIZE(arr) → sizeof(arr) / sizeof(arr[0]) → 40 / 4 = 10
 *
 * 注意: 只对编译期定义大小的数组有效, 指针参数不能用 (sizeof(ptr) = 4)
 */
#define ARRAY_SIZE(arr)                (sizeof(arr) / sizeof((arr)[0]))

/*
 * CLAMP: 将 val 限幅到 [min, max] 范围 (整数版本)
 *
 * 例: CLAMP(1500, 0, 1000)  → 1000
 *     CLAMP(-100, 0, 1000)  → 0
 *     CLAMP(500,  0, 1000)  → 500
 *
 * 注意: val 会被计算两次 (宏展开), 不要传带副作用的表达式如 val++
 */
#define CLAMP(val, min, max)           (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

/*
 * CLAMP_F: 浮点数限幅版本 (同 CLAMP 逻辑, 操作类型为 float)
 */
#define CLAMP_F(val, min, max)         (((val) < (min)) ? (min) : (((val) > (max)) ? (max) : (val)))

/* ========================================================================= */
/*  11. 陀螺仪航向保持 (板载 BMI088, SPI1)                                    */
/* ========================================================================= */

/*
 * BMI088 片选引脚 (低电平选中, 默认高电平不选中)
 * CS_GYRO = PB0, CS_ACC = PA4, SPI1 数据线 SCK/MISO/MOSI = PA5/PA6/PA7
 */
#define IMU_GYRO_CS_PORT                GPIOB
#define IMU_GYRO_CS_PIN                 GPIO_PIN_0
#define IMU_ACC_CS_PORT                 GPIOA
#define IMU_ACC_CS_PIN                  GPIO_PIN_4

/** @brief 陀螺仪灵敏度 — ±2000dps → rad/s per LSB (0.001065264436) */
#define IMU_GYRO_RADPS_PER_LSB          (2000.0f / 32768.0f * 3.14159265358979f / 180.0f)

/** @brief 加速度计灵敏度 — ±3g → m/s^2 per LSB (0.0008974358974) */
#define IMU_ACCEL_MPS2_PER_LSB          (3.0f * 9.80665f / 32768.0f)

/** @brief 启动零偏校准采样数 (200 次 × 5ms = 1s, 上电需整车静止) */
#define IMU_CALIB_SAMPLES               200U

/** @brief 航向积分标称周期 (s) — 与控制频率 200Hz 对应 */
#define IMU_NOMINAL_DT                  0.005f

/** @brief 航向方向符号 — 校正陀螺仪 Z 轴与底盘 Wz 方向一致, 反了改 -1.0f */
#define YAW_DIR_SIGN                    1.0f

/** @brief 摇杆旋转死区 — |norm_wz|≥此值视为"摇杆旋转优先" */
#define HEADING_WZ_DEADZONE             0.02f

/* ---- 航向保持 PID (外环: 航向角→Wz, 输出单位 rad/s 直接进运动学) ---- */
/* 当前为防抖保守值: Kp 偏小 + Kd 阻尼 + Ki=0 (积分易引发极限环振荡)
 * 实车标定: 回正慢→升 Kp; 震荡→降 Kp 或升 Kd; 稳态偏航→才加 Ki */
#define HEADING_PID_KP                  1.0f
#define HEADING_PID_KI                  0.0f
#define HEADING_PID_KD                  0.1f
#define HEADING_PID_MAX_OUT             CHASSIS_MAX_ANGULAR_SPEED_RADPS
#define HEADING_PID_MAX_I               0.2f
#define HEADING_PID_SEP_THRESH          0.5f    /* |误差|>0.5rad 清积分 */
#define HEADING_PID_D_ALPHA             0.1f

/* ========================================================================= */
/*  12. WS2815 灯带 (两条, PWM+DMA 驱动)                                      */
/*                                                                            */
/*  硬件:                                                                      */
/*    A 条 → TIM1_CH1 (PE9), DMA2_Stream5                                    */
/*    B 条 → TIM8_CH1 (PI5), DMA2_Stream2                                    */
/*    TIM 计数时钟 168MHz, ARR=209 → 每 bit 1.25µs (800kHz 协议)              */
/*                                                                            */
/*  ★ 修改灯珠数只需改下面两个 LEN 宏, 帧缓冲自动适配, 无需改代码。            */
/* ========================================================================= */

/** @brief A 条灯珠数量 — TIM1_CH1 (PE9), 按实际灯带修改 */
#define WS2815_STRIP_A_LEN              24U

/** @brief B 条灯珠数量 — TIM8_CH1 (PI5), 按实际灯带修改 */
#define WS2815_STRIP_B_LEN              24U

/* ---- 呼吸灯参数 ---- */

/** @brief 一个呼吸周期 (ms) — 灭→最亮→灭 */
#define WS2815_BREATH_PERIOD_MS         2000U

/** @brief 亮度刷新间隔 (ms) — 20ms = 50Hz, 呼吸平滑且不抢 CPU */
#define WS2815_BREATH_STEP_MS           20U

/** @brief 呼吸基础色 (RGB 0~255) — 实际亮度 = 基础色 × 正弦亮度 */
#define WS2815_BREATH_BASE_R            0U
#define WS2815_BREATH_BASE_G            120U
#define WS2815_BREATH_BASE_B            255U

/* ---- WS2815 任务 (呼吸灯) ---- */

#define PRIO_WS2815                     osPriorityLow           /**< 效果任务, 最低优先级即可 */
#define STACK_WS2815                    1024U                   /**< 1 KiB — 单次编码 + cosf 计算 */

#ifdef __cplusplus
}
#endif

#endif /* __APP_CONFIG_H__ */
