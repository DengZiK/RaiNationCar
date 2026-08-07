# 麦科勒姆轮底盘机器人电控系统 — 设计文档

> **状态**: 等待用户确认后进入代码编写阶段  
> **日期**: 2026-07-26

---

## 1. 项目架构总览

```
┌──────────────────────────────────────────────────────────────────────┐
│                          FreeRTOS 任务调度层                           │
├────────────┬────────────┬────────────┬────────────┬──────────────────┤
│ 遥控接收任务 │ 底盘控制任务 │ 升降控制任务 │ 电磁阀任务   │ 安全监控任务       │
│ (DR16 解析) │ (Mecanum   │ (Position  │ (GPIO 开关) │ (超时/断联保护)    │
│ 200Hz       │  Speed PID)│  PID 100Hz)│ 事件驱动    │ 10Hz              │
├────────────┴────────────┴────────────┴────────────┴──────────────────┤
│                           HAL 硬件抽象层                               │
├──────────┬──────────┬──────────┬──────────┬──────────┬───────────────┤
│  CAN1    │ USART3   │  GPIO    │  TIMx    │  SPI/I2C │  DMA          │
│ (C620电调)│ (遥控器)  │ (电磁阀)  │ (编码器/PWM)│ (IMU等) │ (串口DMA接收)  │
└──────────┴──────────┴──────────┴──────────┴──────────┴───────────────┘
```

### 模块划分

| 模块       | 说明                              | 文件                        |
| ---------- | --------------------------------- | --------------------------- |
| `chassis`  | 麦科勒姆轮运动学解算 + 4路速度PID | `chassis.h` / `chassis.c`   |
| `lift`     | 升降机构位置PID控制               | `lift.h` / `lift.c`         |
| `valve`    | 电磁阀GPIO控制                    | `valve.h` / `valve.c`       |
| `pid`      | 通用PID控制器                     | `pid.h` / `pid.c`           |
| `remote`   | SBUS 遥控器解析 (ISR中完成)       | `remote.h` / `remote.c`     |
| `motor`    | DJI C620电调 CAN通信封装          | `motor.h` / `motor.c`       |
| `watchdog` | 超时保护 / 断联安全               | `watchdog.h` / `watchdog.c` |
| `vofa`     | VOFA+ JustFloat 串口遥测          | `vofa.h` / `vofa.c`         |
| `app`      | FreeRTOS 任务入口与调度           | `freertos.c` + `app_config.h` |

---

## 2. CubeMX 配置要点

### 2.1 系统时钟 (System Core)

| 项              | 配置                          |
| --------------- | ----------------------------- |
| MCU             | STM32F407IGH6                 |
| 时钟源          | HSE 外部 12MHz 晶振           |
| PLLM            | 6 (12MHz / 6 = 2MHz VCO 输入) |
| PLLN            | 168 (2MHz × 168 = 336MHz VCO) |
| PLLP            | 2 (336MHz / 2 = 168MHz)       |
| SYSCLK          | 168 MHz                       |
| APB1 定时器时钟 | 84 MHz                        |
| APB2 定时器时钟 | 168 MHz                       |

> **注意**: 当前固件使用 HSE 外部 12MHz 晶振，PLL 输入为 2MHz，系统时钟为 168MHz。

### 2.2 FreeRTOS 配置

| 项                            | 值        |
| ----------------------------- | --------- |
| Interface                     | CMSIS_V2  |
| TICK_RATE_HZ                  | 1000 Hz   |
| MINIMAL_STACK_SIZE            | 128 words |
| MAX_PRIORITIES                | 16        |
| USE_PREEMPTION                | Enabled   |
| configUSE_MUTEXES             | 1         |
| configUSE_COUNTING_SEMAPHORES | 1         |

### 2.3 外设与引脚分配

#### CAN1 — 底盘电机 & 升降电机通信

| 参数      | 值                       |
| --------- | ------------------------ |
| 模式      | Normal                   |
| 波特率    | 1 Mbps                   |
| SJW       | 1                        |
| BS1       | 10                       |
| BS2       | 3                        |
| Prescaler | 3 (APB1 42MHz → 1Mbps)   |
| RX FIFO0  | 使能中断 (CAN1_RX0_IRQn) |
| TX 邮箱   | FIFO 模式                |

| 引脚 | 功能    |
| ---- | ------- |
| PD0  | CAN1_RX |
| PD1  | CAN1_TX |

#### USART3 — DR16 遥控器接收 (SBUS)

| 参数     | 值                                       |
| -------- | ---------------------------------------- |
| 波特率   | 100000 bps                               |
| 数据位   | 9 (含校验位)                             |
| 停止位   | 2                                        |
| 校验位   | Even                                     |
| 流控     | None                                     |
| 接收方式 | DMA (USART3_RX, Circular, 25字节缓冲区)  |

| 引脚 | 功能      |
| ---- | --------- |
| PC10 | USART3_TX |
| PC11 | USART3_RX |

#### GPIO — 电磁阀控制

| 引脚 | 功能           | 模式                      |
| ---- | -------------- | ------------------------- |
| PE0  | 电磁阀控制信号 | Output Push-Pull, No Pull |

#### GPIO — 其他控制信号

| 引脚 | 功能                 | 说明           |
| ---- | -------------------- | -------------- |
| PE1  | LED 状态指示         | Output         |
| PE2  | 急停信号输入（可选） | Input, Pull-Up |

#### 定时器 (用于PID周期控制 / 编码器输入捕获 — 按需)

| 定时器    | 用途                           | 说明                      |
| --------- | ------------------------------ | ------------------------- |
| TIM6      | 底盘控制周期 (5ms / 200Hz)     | 基本定时器, 中断触发      |
| TIM7      | 升降控制周期 (10ms / 100Hz)    | 基本定时器, 中断触发      |
| TIM2~TIM5 | 编码器模式（如使用非C620方案） | 保留, 不使用CAN反馈时备选 |

---

## 3. 引脚分配汇总表

| 引脚  | 功能        | 方向   | 备注                         |
| ----- | ----------- | ------ | ---------------------------- |
| PD0   | CAN1_RX     | Input  | C620 电调反馈                |
| PD1   | CAN1_TX     | Output | C620 电调控制                |
| PC10  | USART3_TX   | Output | SBUS 遥控器                  |
| PC11  | USART3_RX   | Input  | SBUS 遥控器 (DMA Circular)   |
| PG14  | USART6_TX   | Output | VOFA+ 遥测数据输出           |
| PG9   | USART6_RX   | Input  | VOFA+ (预留, 实际未使用)     |
| PE0   | GPIO_Output | Output | 电磁阀控制信号               |
| PE1   | GPIO_Output | Output | LED 状态指示                 |
| PE2   | GPIO_Input  | Input  | 急停 / 外部保护 **(未实现)** |

---

## 4. 数据流设计

```
DR16遥控器 ──(SBUS 100kbps)──► USART3 DMA Circular Buffer (25 bytes)
                                      │
                                      ▼
                              sbus_decode() 解析
                              ├─ CH1~CH4: 摇杆通道 (200~1800, 中位1000)
                              ├─ CH5: Sa 拨杆 (升降预设位)
                              ├─ CH6: Sb 拨杆 (电磁阀开关)
                              └─ CH7~CH10: 辅助通道
                                      │
                    ┌─────────────────┼─────────────────┐
                    ▼                 ▼                  ▼
            chassis_control()   lift_control()    valve_control()
            (麦科勒姆解算)       (位置PID设定)      (开关映射)
                    │                 │                  │
                    ▼                 ▼                  ▼
            4× 速度PID         1× 位置PID          GPIO Write
                    │                 │
                    ▼                 ▼
            CAN1 Tx (1-4号)     CAN1 Tx (5号)
            C620 电流指令        C620 电流指令
                    │                 │
           ┌────────┴────────┐        │
           ▼    ▼    ▼    ▼  │        ▼
          M1   M2   M3   M4  │       M5
          (底盘四轮)          │    (升降电机)
                              │
                    CAN1 Rx 反馈
                    (转速 / 位置 / 温度 / 电流)
                              │
                    ┌─────────┴─────────┐
                    ▼                   ▼
             PID 反馈更新          超时检测
                                    │
                              ┌─────┴──────┐
                              │ 超时 → 急停  │
                              │ CAN输出清零  │
                              └────────────┘
```

---

## 5. 数据结构定义

### 5.1 通用PID结构体

```c
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral_limit;   // 积分限幅
    float output_limit;     // 输出限幅 (±16384 for C620)
    float setpoint;
    float measured;
    float integral;
    float prev_error;
    float output;
} PID_Controller_t;
```

### 5.2 电机CAN数据结构

```c
// C620 电调控制帧 (ID: 0x200 + 电调组)
typedef struct {
    int16_t current[4];     // 4路电流值, 范围 ±16384 (对应 ±20A)
} Motor_CAN_Tx_t;

// C620 电调反馈帧 (ID: 0x200 + 电调ID)
typedef struct {
    uint16_t angle;         // 机械角度 0-8191
    int16_t  speed_rpm;     // 转速 rpm
    int16_t  torque_current;// 实际电流
    uint8_t  temperature;   // 温度 ℃
} Motor_CAN_Rx_t;
```

### 5.3 遥控器数据结构

```c
// SBUS 25字节帧, 10通道, 每通道11bit
typedef struct {
    int16_t right_x;    // CH1: 右摇杆左右
    int16_t right_y;    // CH2: 右摇杆上下
    int16_t left_y;     // CH3: 左摇杆上下
    int16_t left_x;     // CH4: 左摇杆左右
    int16_t ch5;        // Sa 拨杆
    int16_t ch6;        // Sb 拨杆
    int16_t ch7;        // Sc 拨杆
    int16_t ch8;        // Sd 拨杆
    int16_t ch9;        // Se 拨杆
    int16_t ch10;       // Sg 拨杆
    uint8_t connected;  // 1=连接正常, 0=信号丢失(失控保护)
} Remote_Data_t;
```

> **SBUS 协议要点**:
> - 帧长 25 字节，波特率 100kbps，9 数据位 + Even 校验 + 2 停止位
> - 帧头 `0x0F`，帧尾 `0x00`
> - 原始值域 0~2047，经 `sbus_to_rc()` 映射到 200~1800（中位 1000，死区 ±30）
> - 解析在 USART3 IDLE 中断中完成，不依赖 FreeRTOS 信号量

### 5.4 底盘状态结构体

```c
typedef struct {
    float target_vx;        // 目标 X 速度 (mm/s)
    float target_vy;        // 目标 Y 速度 (mm/s)
    float target_vw;        // 目标旋转角速度 (rad/s × 1000)
    float wheel_rpm[4];     // 4轮目标转速 rpm
    float wheel_rpm_fb[4];  // 4轮反馈转速 rpm
    PID_Controller_t wheel_pid[4];  // 4路速度PID
    int16_t current_out[4]; // 电流输出值
} Chassis_t;
```

### 5.5 升降机构状态结构体

```c
typedef struct {
    float target_position;   // 目标位置 (encoder count 或 角度)
    float current_position;  // 当前位置反馈
    PID_Controller_t position_pid;
    int16_t current_out;     // 电流输出值
} Lift_t;
```

---

## 6. FreeRTOS 任务设计

| 任务名          | 优先级            | 周期/触发      | 功能                                    |
| --------------- | ----------------- | -------------- | --------------------------------------- |
| `Task_Watchdog` | High (最高)       | 100ms (10Hz)   | 遥控器/电机超时检测、急停、LED 状态指示 |
| `Task_Remote`   | AboveNormal       | 50ms (占位)    | SBUS 解析已在 ISR 完成，此任务仅占位    |
| `Task_Chassis`  | Normal            | 5ms (200Hz)    | 麦科勒姆解算 + 4路速度PID + CAN发送     |
| `Task_Lift`     | Normal            | 10ms (100Hz)   | 位置PID计算 + CAN发送                   |
| `Task_Valve`    | BelowNormal (最低)| 20ms (50Hz)    | 电磁阀GPIO开关控制                      |

> **设计要点**:
> - SBUS 解析在 `USART3_IRQHandler` → `RC_UART_IRQHandler()` 中完成，不经过 FreeRTOS 任务
> - 遥控器超时检测已合并到 `Task_Watchdog`，`Task_Remote` 仅做 `osDelay(50)` 占位
> - `Task_Chassis` 和 `Task_Lift` 同级优先级，各通过 `vTaskDelayUntil` 主动让出 CPU，不会冲突
> - 不使用 FreeRTOS 队列/信号量，全通过全局变量 + ARM M4 原子读写通信

---

## 7. 麦科勒姆轮运动学

### 解算公式

```
V_wheel_1 =  Vx - Vy - (Lx + Ly) * Wz    // 前右 (FR)
V_wheel_2 =  Vx + Vy + (Lx + Ly) * Wz    // 前左 (FL)
V_wheel_3 = -Vx + Vy - (Lx + Ly) * Wz    // 后左 (RL)
V_wheel_4 = -Vx - Vy + (Lx + Ly) * Wz    // 后右 (RR)

其中: Lx = 轮距/2, Ly = 轴距/2
      V_wheel_n 为各轮线速度 (mm/s)，除以轮子周长可换算为 rpm
```

### 轮子编号 (俯视图)

```
     前
   ┌──────┐
   │ 2  1 │   ← 前左(2)  前右(1)
   │      │
   │ 3  4 │   ← 后左(3)  后右(4)
   └──────┘
```

---

## 8. 安全逻辑

1. **遥控器断联超时**: 连续 > 100ms 未收到 SBUS 数据 → 所有电机输出置零 + 关闭电磁阀
2. **CAN通信超时**: 任意电机 > 200ms 无反馈 → 所有电机输出置零
3. **急停信号 (PE2)**: 硬件接口已预留，**当前代码未实现** — 待后续版本加入
4. **输出限幅**: PID输出硬钳位到 C620 电流范围 ±16384
5. **积分分离/抗饱和**: 大偏差时清除积分项, 输出已达限幅时停止积分
6. **LED 状态指示 (PE1)**:
   - 常亮: 系统正常
   - 500ms 周期闪烁: 遥控器信号丢失
   - 200ms 周期闪烁: 电机 CAN 通信超时
   - 灭: 急停 (ESTOP)

---

## 9. 待用户确认事项

1. 请确认 MCU 型号（示例中使用 STM32F407IGH6，实际是否有差异）
2. 请确认电机编号/布局方向、轮距轴距参数
3. 请提供遥控器数据解析的现有文件（`remote.h` / `remote.c`）
4. 请提供现有的 PID 控制文件（`pid.h` / `pid.c`）
5. 升降机构传动参数（减速比、丝杆导程、编码器分辨率）
6. 电磁阀是 高电平触发 还是 低电平触发？电压等级？
7. 是否需要添加 IMU (姿态传感器) 用于闭环航向控制？

---

## 10. 目录结构规划

```
NationalCar/
├── NatonalCarSet/
│   ├── NatonalCarSet.ioc            ← CubeMX 项目文件
│   ├── CMakeLists.txt
│   ├── STM32F407XX_FLASH.ld         ← 链接脚本
│   ├── startup_stm32f407xx.s        ← 启动文件
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h
│   │   │   ├── can.h
│   │   │   ├── usart.h
│   │   │   ├── gpio.h
│   │   │   ├── dma.h
│   │   │   ├── stm32f4xx_it.h
│   │   │   ├── stm32f4xx_hal_conf.h
│   │   │   └── FreeRTOSConfig.h
│   │   └── Src/
│   │       ├── main.c               ← 入口 + 应用初始化 + HAL 回调
│   │       ├── freertos.c           ← FreeRTOS 任务定义与入口
│   │       ├── can.c / usart.c / gpio.c / dma.c
│   │       ├── stm32f4xx_it.c       ← 中断向量 (含 USART3_IRQHandler)
│   │       ├── system_stm32f4xx.c
│   │       └── stm32f4xx_hal_msp.c
│   ├── Drivers/                     ← STM32F4 HAL 库 (CubeMX 生成)
│   ├── Middlewares/                 ← FreeRTOS 源码 (CubeMX 生成)
│   ├── user/
│   │   ├── app/
│   │   │   ├── app_config/
│   │   │   │   └── app_config.h     ← 集中参数配置
│   │   │   ├── chassis/
│   │   │   │   ├── chassis.h
│   │   │   │   └── chassis.c        ← 麦科勒姆逆运动学 + 4路速度PID
│   │   │   ├── lift/
│   │   │   │   ├── lift.h
│   │   │   │   └── lift.c           ← 升降位置PID + 拨杆预设位
│   │   │   └── watchdog/
│   │   │       ├── watchdog.h
│   │   │       └── watchdog.c       ← 安全监控 + LED 状态
│   │   ├── bsp/
│   │   │   ├── motor/
│   │   │   │   ├── motor.h
│   │   │   │   └── motor.c          ← C620 CAN 通信 + 圈数累积
│   │   │   ├── remote/
│   │   │   │   ├── remote.h
│   │   │   │   └── remote.c         ← SBUS 25字节帧解析 (ISR)
│   │   │   └── valve/
│   │   │       ├── valve.h
│   │   │       └── valve.c          ← 电磁阀 GPIO 控制
│   │   └── algo/
│   │       ├── pid/
│   │       │   ├── pid.h
│   │       │   └── pid.c            ← 通用 PID (积分分离+微分滤波)
│   │       └── vofa/
│   │           ├── vofa.h
│   │           └── vofa.c           ← VOFA+ JustFloat 遥测
│   └── MDK-ARM/                     ← Keil MDK 工程
├── DESIGN.md                        ← 本文档
├── FreeRTOS_任务设计详解.md         ← FreeRTOS 任务详解
├── VOFA_使用说明.md                 ← VOFA+ 调试遥测说明
└── propmt.md                        ← 需求原文
```

---

> **下一步**: 请审阅以上设计，确认或修改各项参数后，我将开始编写具体业务代码。
