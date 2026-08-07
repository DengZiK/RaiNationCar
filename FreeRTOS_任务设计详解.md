# FreeRTOS 任务设计详解 — NationalCar 麦科勒姆轮底盘

> **文件**: `NatonalCarSet/Core/Src/freertos.c`  
> **MCU**: STM32F407IGH6 (168MHz)  
> **RTOS**: FreeRTOS CMSIS_V2, 1000Hz Tick, heap_4, 15KB heap  
> **日期**: 2026-07-26

---

## 目录

1. [文件结构总览](#1-文件结构总览)
2. [任务一：TaskWatchdog — 安全监控](#2-taskwatchdog--安全监控-10hz)
3. [任务二：TaskRemote — 遥控器辅助](#3-taskremote--遥控器辅助-50ms)
4. [任务三：TaskChassis — 底盘控制](#4-taskchassis--底盘控制-200hz)
5. [任务四：TaskLift — 升降控制](#5-tasklift--升降控制-100hz)
6. [任务五：TaskValve — 电磁阀控制](#6-taskvalve--电磁阀控制-50hz)
7. [defaultTask — 空闲占位](#7-defaulttask--空闲占位)
8. [FreeRTOS 核心知识点](#8-freertos-核心知识点)
9. [调度时序图](#9-调度时序图)
10. [与 CubeMX 的关系](#10-与-cubemx-的关系)

---

## 1. 文件结构总览

### 1.1 代码分区

```
freertos.c
├── [CubeMX 生成]  includes / typedef / define / 变量
├── [CubeMX 生成]  defaultTask 句柄与属性
├── [USER CODE]    5 个自定义任务句柄与属性    ← 手动编写
├── [CubeMX 生成]  MX_FREERTOS_Init() 框架
│   └── [USER CODE] osThreadNew() × 5          ← 手动添加
├── [CubeMX 生成]  StartDefaultTask()         ← CubeMX 自动
└── [USER CODE]    5 个任务入口函数             ← 手动编写
```

CubeMX 的 `USER CODE BEGIN/END` 注释对保证了重新生成代码时自定义部分不会被覆盖。

### 1.2 任务速查表

| 任务 | 优先级 | 周期 | 栈 | 核心函数 | 功能 |
|------|--------|------|-----|---------|------|
| **TaskWatchdog** | `osPriorityHigh` (最高) | 100ms (10Hz) | 1024B | `Watchdog_Update()` | 安全监控 → 急停 |
| **TaskRemote** | `osPriorityAboveNormal` | 50ms | 1024B | `osDelay(50)` | 遥控器辅助 (SBUS 解析在 ISR) |
| **TaskChassis** | `osPriorityNormal` | 5ms (200Hz) | 2048B | `Chassis_Update()` | 底盘运动学+PID+CAN |
| **TaskLift** | `osPriorityNormal` | 10ms (100Hz) | 2048B | `Lift_Update()` | 升降位置PID+CAN |
| **TaskValve** | `osPriorityBelowNormal` (最低) | 20ms (50Hz) | 512B | `Valve_Process()` | 读拨杆 → GPIO |

---

## 2. TaskWatchdog — 安全监控 (10Hz)

### 2.1 创建代码

```c
/* 属性定义 */
osThreadId_t taskWatchdogHandle;
const osThreadAttr_t taskWatchdog_attributes = {
    .name = "TaskWatchdog",
    .stack_size = STACK_WATCHDOG * 4,   // 256 × 4 = 1024 bytes
    .priority = PRIO_WATCHDOG,           // osPriorityHigh
};

/* 创建 (在 MX_FREERTOS_Init 中) */
taskWatchdogHandle = osThreadNew(StartTaskWatchdog, NULL, &taskWatchdog_attributes);
```

### 2.2 入口函数

```c
void StartTaskWatchdog(void *argument)
{
    (void)argument;

    osDelay(500);  // ★ 等 500ms：让系统、外设、CAN 先初始化完毕

    TickType_t last_wake = osKernelGetTickCount();

    for (;;) {
        Watchdog_Update();  // 检查遥控器超时 + 电机超时 + LED 指示
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000U / WATCHDOG_FREQ_HZ));
        // 1000 / 10 = 100ms 周期
    }
}
```

### 2.3 功能详解

`Watchdog_Update()` 每 100ms 执行一次：

```
1. remote_control_watchdog() → 更新 rc_data.connected
2. 检查 rc_data.connected == 0? → 急停 + 关阀
3. 检查 5 个电机 CAN 超时?  → 急停
4. 根据状态控制 LED: 常亮 / 500ms闪 / 200ms闪 / 灭
```

### 2.4 为什么优先级最高

安全任务必须在任何情况下抢占 CPU：
- 底盘 200Hz 任务跑飞 → Watchdog 仍能急停
- CAN 中断风暴 → Watchdog 仍能检测超时
- 如果 Watchdog 优先级低于 Chassis，底盘死循环会导致永远无法急停

---

## 3. TaskRemote — 遥控器辅助 (50ms)

### 3.1 入口函数

```c
void StartTaskRemote(void *argument)
{
    (void)argument;
    for (;;) {
        osDelay(50);
    }
}
```

### 3.2 为什么是空的

**SBUS 解析完全在 ISR 中完成**，不需要任务参与：

```
USART3 DMA Circular (25 bytes)
    │
    └─ IDLE 中断触发
        └─ USART3_IRQHandler()
            └─ RC_UART_IRQHandler()
                ├─ SBUS_CopyFrameFromRing()  // DMA 环形缓冲重组
                └─ sbus_decode()             // 位拼接 10 通道 → rc_data
```

遥控器超时检测（`remote_control_watchdog()`）已合并到 TaskWatchdog 中，所以这个任务目前只做 `osDelay(50)` 占位。

> **保留原因**: 后续可能需要加入遥控器信号的二次滤波、通道映射切换等功能。

---

## 4. TaskChassis — 底盘控制 (200Hz)

### 4.1 创建代码

```c
osThreadId_t taskChassisHandle;
const osThreadAttr_t taskChassis_attributes = {
    .name = "TaskChassis",
    .stack_size = STACK_CHASSIS * 4,   // 512 × 4 = 2048 bytes
    .priority = PRIO_CHASSIS,           // osPriorityNormal
};
```

### 4.2 入口函数

```c
void StartTaskChassis(void *argument)
{
    (void)argument;

    TickType_t last_wake = osKernelGetTickCount();

    for (;;) {
        Chassis_Update();  // 核心业务
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHASSIS_PERIOD_MS));
        // CHASSIS_PERIOD_MS = 5ms → 200Hz
    }
}
```

### 4.3 `Chassis_Update()` 每 5ms 执行流程

```
1. 安全检查: rc_data.connected? Motor_IsTimeout(4 motors)?
2. 读取遥控器: right_y→Vx, right_x→Vy, left_x→Wz
   (SBUS 200~1800 → 归一化 ±1 → 乘最大速度 m/s)
3. 死区判断 (< 2% → 视为 0)
4. 麦科勒姆逆运动学: 4 个轮子转速 = f(Vx, Vy, Wz)
5. 速度 PID × 4 轮 → 计算电流指令
6. 输出限幅 → g_motor.current_setpoint[]
7. Motor_SendChassisCurrent() → CAN ID 0x200
```

### 4.4 为什么是 200Hz

| 因素 | 说明 |
|------|------|
| M3508 C620 控制频率 | 电调接收 1kHz，但实际响应带宽约 100~200Hz |
| CAN 总线负载 | 1Mbps，0x200 帧占 108 bits，200Hz 仅占 2.2% 带宽 |
| PID 控制理论 | 速度环通常 100~500Hz，200Hz 满足大部分需求 |
| CPU 负载 | 5ms 周期留给其他任务 95% 时间 |

---

## 5. TaskLift — 升降控制 (100Hz)

### 5.1 入口函数

```c
void StartTaskLift(void *argument)
{
    (void)argument;

    TickType_t last_wake = osKernelGetTickCount();

    for (;;) {
        Lift_Update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LIFT_PERIOD_MS));
        // LIFT_PERIOD_MS = 10ms → 100Hz
    }
}
```

### 5.2 `Lift_Update()` 每 10ms 执行流程

```
1. 安全检查: rc_data.connected? Motor_IsTimeout(MOTOR_LIFT)?
2. 更新当前位置: Motor_GetTotalRevolutions() × 8192
3. 读取 ch5 (Sa 拨杆): <500→UP, 500~1500→MID, >1500→DOWN
4. 边沿检测 → 更新目标位置 preset
5. 位置 PID → 计算电流指令
6. Motor_SendLiftCurrent() → CAN ID 0x1FF
```

### 5.3 为什么是 100Hz

| 因素 | 说明 |
|------|------|
| 机械惯性大 | 升降机构减速比 19:1，响应比底盘慢 |
| 位置环特性 | 位置环带宽通常低于速度环，100Hz 足够 |
| ch5 拨杆变化 | 人手指拨动 Sa 开关的频率远低于 100Hz |

---

## 6. TaskValve — 电磁阀控制 (50Hz)

### 6.1 入口函数

```c
void StartTaskValve(void *argument)
{
    (void)argument;

    TickType_t last_wake = osKernelGetTickCount();

    for (;;) {
        Valve_Process();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
        // 20ms → 50Hz
    }
}
```

### 6.2 `Valve_Process()` 每 20ms 执行流程

```
1. 读取 rc_data.ch6 (Sb 拨杆)
2. >1500 → DOWN(开), <500 → UP(关), 其他 → MID(保持)
3. 边沿检测: 状态变化才动作 (防反复触发)
4. HAL_GPIO_WritePin(PE0, ON/OFF)
```

### 6.3 为什么优先级最低

电磁阀是低速开关量控制，50Hz 绰绰有余。即使被底盘任务延迟 5ms 也不影响功能。

---

## 7. defaultTask — 空闲占位

```c
void StartDefaultTask(void *argument)
{
    (void)argument;
    for (;;) {
        osDelay(1);
    }
}
```

这是 CubeMX 在 Tasks 选项卡自动生成的任务，项目中未使用，仅保留占位。**删掉不影响系统运行**（FreeRTOS 有内置 Idle Task 处理空闲时间）。

---

## 8. FreeRTOS 核心知识点

### 8.1 CMSIS_V2 API — 任务创建

项目使用 CMSIS_V2 封装层而非原生 FreeRTOS API：

```c
// CMSIS_V2 风格 (本项目使用)
osThreadId_t handle = osThreadNew(TaskFunc, NULL, &attributes);

// 等价的原生 FreeRTOS
BaseType_t ret = xTaskCreate(TaskFunc, "name", stack, NULL, priority, &handle);
```

**osThreadAttr_t 结构体：**

```c
const osThreadAttr_t task_attributes = {
    .name       = "TaskChassis",        // 调试用名称 (IDE 可显示)
    .stack_size = 512 * 4,              // 栈大小: 字(word) × 4 = 字节
    .priority   = osPriorityNormal,     // CMSIS 优先级枚举
    // 可选:
    // .attr_bits = osThreadDetached,   // 分离模式
    // .cb_mem / .cb_size / .stack_mem / .stack_size  // 静态分配
};
```

> **陷阱**: `stack_size` 的单位是**字 (word)**，不是字节。ARM Cortex-M4 一个字 = 4 bytes，所以 `512 × 4 = 2048 bytes`。

### 8.2 优先级与抢占

本项目优先级配置：

```
osPriorityHigh           TaskWatchdog ───────── 最高
    ↑
osPriorityAboveNormal    TaskRemote
    ↑
osPriorityNormal         TaskChassis, TaskLift ─ 同级
    ↑
osPriorityBelowNormal    TaskValve ──────────── 最低
    ↑
osPriorityLow            (idle task)
```

**抢占规则：**

1. 高优先级就绪 → **立即抢占**低优先级
2. 同级优先级 → **时间片轮转** (configUSE_TIME_SLICING=1)
3. 中断 (ISR) → **永远抢占** 所有任务

**本项目例子：**
- TaskChassis(200Hz) 和 TaskLift(100Hz) 同级，各有 `vTaskDelayUntil` 主动让出 CPU，不会冲突
- TaskWatchdog 可在任何时候抢占底盘任务执行急停

### 8.3 vTaskDelayUntil — 精确周期定时

```c
TickType_t last_wake = osKernelGetTickCount();

for (;;) {
    // ... 业务代码 ...
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
}
```

**执行时序：**

```
Time:  0ms    5ms   10ms   15ms   20ms
       │      │     │      │      │
       ▼      ▼     ▼      ▼      ▼
       [Run]  [Run] [Run]  [Run]  [Run]
              ▲            ▲
              │            │
         业务耗时2ms   业务耗时3ms
         睡眠3ms       睡眠2ms
```

| 特性 | `vTaskDelayUntil` | `osDelay` |
|------|-------------------|-----------|
| 定时方式 | **绝对时间** (从上次唤醒起算) | **相对时间** (从调用起算) |
| 周期漂移 | 无 (自动补偿) | 会累积 (业务耗时叠加) |
| 适用场景 | 固定频率控制循环 | 一般延时等待 |

**为什么本项目必须用 vTaskDelayUntil：**

假设用 `osDelay(5)`：
```
周期1: Run(3ms) + Delay(5ms) = 8ms → 周期漂了 3ms
周期2: Run(2ms) + Delay(5ms) = 7ms → 漂了 5ms
...累积下去，200Hz 变成 ~150Hz
```

`vTaskDelayUntil` 会在内部自动补偿：如果本次延迟了，下次少睡一点，保持绝对时间节点不变。

### 8.4 栈大小估算

| 任务 | 栈大小 | 估算依据 |
|------|--------|---------|
| Watchdog | 256 words (1024B) | 简单逻辑 + 无浮点运算 |
| Remote | 256 words (1024B) | 空循环，仅占位 |
| Chassis | 512 words (2048B) | 浮点运算 + 运动学矩阵 + PID ×4 |
| Lift | 512 words (2048B) | 浮点运算 + PID + 位置计算 |
| Valve | 128 words (512B) | 简单 GPIO 操作 |

**栈溢出检测方法：**

```c
// 在任务创建后检查剩余栈空间 (调试用)
UBaseType_t uxHighWaterMark;
uxHighWaterMark = uxTaskGetStackHighWaterMark(taskChassisHandle);
// 返回值接近 0 → 危险，需增大栈
```

### 8.5 configTOTAL_HEAP_SIZE = 15360 bytes

FreeRTOS 使用 heap_4 内存管理，15KB 堆分配给了：

```
任务栈:
  defaultTask  128×4 = 512B
  Watchdog     256×4 = 1024B
  Remote       256×4 = 1024B
  Chassis      512×4 = 2048B
  Lift         512×4 = 2048B
  Valve        128×4 = 512B
  ────────────────────────
  任务栈合计:         7168B

剩余堆 (~8KB): 给 FreeRTOS 内核结构 (TCB、链表、定时器等)
```

### 8.6 中断与任务的通信

本项目**不使用 FreeRTOS 队列/信号量/互斥量**，数据流全部通过**全局变量 + ISR 直接更新**：

```
┌──────────────────────────────────────────────────────┐
│ ISR (USART3 IDLE)                                    │
│   RC_UART_IRQHandler()                               │
│   → sbus_decode() → rc_data.right_x, .ch5, ...       │
├──────────────────────────────────────────────────────┤
│ ISR (CAN1 RX0)                                       │
│   HAL_CAN_RxFifo0MsgPendingCallback()                │
│   → Motor_RxCallback() → g_motor.feedback[].speed_rpm│
├──────────────────────────────────────────────────────┤
│ 任务层                                               │
│   Chassis_Update()  → 读 rc_data + g_motor           │
│   Lift_Update()     → 读 rc_data + g_motor           │
│   Watchdog_Update() → 读 rc_data.connected            │
└──────────────────────────────────────────────────────┘
```

**为什么不需要信号量 / 临界区：**

| 共享数据 | 类型 | 原子性 |
|---------|------|--------|
| `rc_data.ch5` | `int16_t` (2 bytes) | ARM M4 天然原子读写 |
| `g_motor.feedback[].speed_rpm` | `int16_t` | 天然原子 |
| `rc_data.connected` | `uint8_t` | 天然原子 |
| `g_motor.current_setpoint[]` | `int16_t[]` | 每个元素独立原子 |

> ARM Cortex-M4 对 **对齐的 8/16/32-bit** 内存访问是原子的。没有超过 32-bit 的多字节结构体需要保护，所以直接读全局变量是安全的。

### 8.7 FreeRTOSConfig.h 关键参数

| 参数 | 本项目值 | 说明 |
|------|---------|------|
| `configTICK_RATE_HZ` | 1000 | 1ms 一个 tick，定时精度 1ms |
| `configMAX_PRIORITIES` | 56 | 最大优先级数 (CMSIS 标准) |
| `configTOTAL_HEAP_SIZE` | 15360 | 总堆大小 (heap_4) |
| `configUSE_PREEMPTION` | 1 | 抢占式调度 |
| `configUSE_TIME_SLICING` | 1 | 同级任务时间片轮转 |
| `configUSE_MUTEXES` | 1 | 启用互斥量 |
| `configMINIMAL_STACK_SIZE` | 128 | 最小任务栈 (words) |

---

## 9. 调度时序图

```
时间轴 →  0    5    10   15   20   25   30   35   40   45   50ms
          │    │    │    │    │    │    │    │    │    │    │
Watchdog  │··········[W]··············│··········[W]··············
(10Hz)    │                          │
          │                          │
Remote    │·····[R]········│·········│·····[R]········│·········
(50ms)    │                │         │                │
          │                │         │                │
Chassis   │[C][C][C][C][C][C][C][C][C][C][C][C][C][C][C][C][C]
(200Hz)   │▲ ▲ ▲ ▲ ▲ ▲ ▲ ▲ ▲ ▲ ▲ ▲ ▲
          │
Lift      │[L]   [L]   [L]   [L]   [L]   [L]   [L]   [L]   [L]
(100Hz)   │
          │
Valve     │[V]         [V]         [V]         [V]         [V]
(50Hz)    │

[C] = Chassis_Update()  ~2-3ms
[L] = Lift_Update()     ~1ms
[W] = Watchdog_Update() <0.5ms
[R] = Remote 空循环      <0.1ms
[V] = Valve_Process()   <0.1ms

CPU 负载估算: Chassis(200×2.5ms) + Lift(100×1ms) + Watchdog(10×0.5ms) + ...
            = ~600ms/s ≈ 60% CPU
            余量 40% → 健康
```

---

## 10. 与 CubeMX 的关系

### 10.1 CubeMX 生成了什么

| 生成内容 | 位置 |
|---------|------|
| `MX_FREERTOS_Init()` 函数骨架 | `freertos.c` |
| `defaultTask` 句柄、属性、入口函数 | `freertos.c` |
| FreeRTOS 初始化调用 | `main.c` 中 `osKernelInitialize()` + `MX_FREERTOS_Init()` + `osKernelStart()` |
| `FreeRTOSConfig.h` | `Core/Inc/` |

### 10.2 手动添加了什么

| 手动内容 | 保护机制 |
|---------|---------|
| 5 个任务属性定义 | 写在 CubeMX 生成的变量定义区域外 |
| `osThreadNew()` × 5 | 写在 `/* USER CODE BEGIN RTOS_THREADS */` 内 |
| 5 个任务入口函数 | 写在 `/* USER CODE BEGIN Application */` 内 |

CubeMX 重新生成代码时，`USER CODE` 区域内容**完全保留**，不会被覆盖。

### 10.3 CubeMX Tasks 选项卡 vs 手动创建

| 方式 | 优点 | 缺点 |
|------|------|------|
| **CubeMX Tasks 选项卡** | GUI 配置，自动生成代码 | 不灵活，修改后须重新生成 |
| **手动 osThreadNew (本项目)** | 完全控制，参数集中管理 (`app_config.h`) | 需要理解 API |

本项目选择手动方式，因为优先级、栈大小等参数统一定义在 `app_config.h`，一目了然，无需打开 CubeMX 修改。

---

## 附录 A：参数速查

```c
// 任务频率
CHASSIS_CONTROL_FREQ_HZ  200    // 底盘控制频率
LIFT_CONTROL_FREQ_HZ     100    // 升降控制频率
WATCHDOG_FREQ_HZ          10    // 安全监控频率

// 任务栈 (words)
STACK_WATCHDOG            256   // = 1024 bytes
STACK_REMOTE              256   // = 1024 bytes
STACK_CHASSIS             512   // = 2048 bytes
STACK_LIFT                512   // = 2048 bytes
STACK_VALVE               128   // = 512 bytes

// 任务优先级 (CMSIS_V2)
PRIO_WATCHDOG    osPriorityHigh           // 最高: 安全第一
PRIO_REMOTE      osPriorityAboveNormal
PRIO_CHASSIS     osPriorityNormal         // 同优先级: 时间片轮转
PRIO_LIFT        osPriorityNormal
PRIO_VALVE       osPriorityBelowNormal    // 最低: 低速开关量
```

## 附录 B：相关文件

| 文件 | 说明 |
|------|------|
| `Core/Src/freertos.c` | 本文讲解的任务定义与入口 |
| `Core/Src/main.c` | `osKernelInitialize/Start` + 外设初始化 |
| `user/app/app_config/app_config.h` | 所有任务参数宏定义 |
| `user/app/chassis/chassis.c` | `Chassis_Update()` 具体实现 |
| `user/app/lift/lift.c` | `Lift_Update()` 具体实现 |
| `user/app/watchdog/watchdog.c` | `Watchdog_Update()` 具体实现 |
| `user/bsp/valve/valve.c` | `Valve_Process()` 具体实现 |
| `Core/Inc/FreeRTOSConfig.h` | FreeRTOS 内核配置 |
