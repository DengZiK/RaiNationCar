# NatonalCarSet — 智能小车控制系统

基于 **STM32F407 + FreeRTOS** 的轮式机器人设计系统。

系统以遥控器（SBUS）为输入，通过 CAN 总线驱动 4 个麦克纳姆轮底盘电机与 1 个升降机构电机，并集成了陀螺仪航向保持、气动电磁阀控制、安全监控与灯带指示等模块。

---

## 目录

- [主要特性](#主要特性)
- [硬件平台](#硬件平台)
- [系统架构](#系统架构)
  - [FreeRTOS 任务](#freertos-任务)
  - [控制环路](#控制环路)
- [遥控器操作说明](#遥控器操作说明)
- [构建与烧录](#构建与烧录)
- [参数配置](#参数配置)
- [故障诊断与状态指示](#故障诊断与状态指示)
- [目录结构](#目录结构)
- [相关文档](#相关文档)

---

## 主要特性

| 功能 | 说明 |
|:-----|:-----|
| **全向底盘** | 4 路麦克纳姆轮，摇杆直控 `Vx / Vy / Wz`，含加减速限幅平滑 |
| **底盘速度环** | 200Hz，4 轮独立速度 PID（积分分离 + 微分滤波）驱动 M3508 |
| **升降机构** | 100Hz 串级 PID（位置外环 → 速度内环 → 电流），拨杆三档 + 摇杆微调 |
| **航向保持** | 板载 BMI088 陀螺仪积分航向，CH10 开关开启航向锁定 |
| **气动机构** | 4 路电磁阀，遥控器辅助通道独立控制 |
| **安全监控** | 遥控失联 / 电机 CAN 超时 / 任务卡死检测，故障码跨复位保留 |
| **状态指示** | PE1 LED 状态灯 + 双路 WS2815 灯带（呼吸灯） |
| **参数集中管理** | 所有可调参数集中在 `app_config.h`，无需改业务代码 |

---

## 硬件平台

| 部件 | 型号 / 说明 |
|:-----|:-----------|
| 主控 | STM32F407IGH6 @ 168MHz（UFBGA176，CubeMX 工程 `NatonalCarSet.ioc`） |
| 电机 ×5 | 大疆 M3508（减速比 19.2:1），14-bit 磁编码器（8192 计数/圈） |
| 电调 ×5 | 大疆 C620，CAN ID 通过 DIP 拨码设置（底盘 1~4，升降 5） |
| 底盘 | 4 麦克纳姆轮 + 全向运动学解算 |
| 升降机构 | 同步带传动，位置换算到 mm，软限位保护 |
| 遥控器 | AT9S，SBUS 协议 @ USART3（DMA 循环 + IDLE 中断），10 通道 |
| IMU | 板载 BMI088 @ SPI1（SCK=PA5, MISO=PA6, MOSI=PA7；CS_GYRO=PB0, CS_ACC=PA4） |
| 电磁阀 ×4 | PB12~PB15，高电平通电开阀 |
| 灯带 ×2 | WS2815：A 条 TIM1_CH1(PE9)/DMA2_Stream5，B 条 TIM8_CH1(PI5)/DMA2_Stream2 |
| 调试串口 | USART6（VOFA+ 波形上位机） |
| 状态 LED | PE1 |

**CAN 通信**：1 Mbps（APB1 42MHz ÷ 3 ÷ (1+10+3)）。控制帧 `0x200`（底盘 4×int16 电流）/`0x1FF`（升降）；反馈帧 `0x201~0x205`（角度、转速、电流、温度），每电机 1kHz。详见 [docs/DESIGN_MOTOR_CAN_MAPPING.md](docs/DESIGN_MOTOR_CAN_MAPPING.md)。

---

## 系统架构

```
STM32F407IGH6 @ 168MHz
│
├─ 启动流程: HAL_Init → SystemClock → MX 外设 → 应用层 Init → FreeRTOS
│
├─ USART3 (SBUS 遥控) ──── ISR 解析 → rc_data
├─ CAN1 ────────────────── RX 中断 → g_motor.feedback[] 反馈
│                             TX 任务 → 电流指令 (0x200 / 0x1FF)
├─ SPI1 (BMI088) ───────── 200Hz 读取 + 航向积分
├─ TIM1/TIM8 (WS2815) ──── PWM+DMA 双灯带
├─ GPIO ────────────────── 电磁阀 (PB12~15) / 状态 LED (PE1)
└─ FreeRTOS ────────────── 6 个任务 (见下表)
```

### FreeRTOS 任务

| 任务 | 频率 | 优先级 | 栈 (B) | 职责 |
|:-----|:----:|:------:|:------:|:-----|
| `Watchdog_task` | 10Hz | High | 1024 | 安全监控、LED 指示、喂 IWDG |
| `Remote_task` | 50Hz | AboveNormal | 1024 | 遥控信号丢失检测 |
| `Chassis_task` | 200Hz | Normal | 2048 | 运动学 + 4 路速度 PID + IMU 航向积分 |
| `Lift_task` | 100Hz | Normal | 2048 | 升降串级 PID |
| `Valve_task` | 50Hz | Normal | 512 | 4 路电磁阀拨杆边沿检测 |
| `WS2815_task` | 50Hz | Low | 1024 | 灯带呼吸灯效果 |

优先级/栈大小在 [app_config.h](user/app/app_config/app_config.h) 第 9 节集中管理。

### 控制环路

**底盘 (200Hz)**：

```
遥控器摇杆 → Vx/Vy/Wz 目标 → 加减速限幅 → 麦轮逆运动学
  → 各轮目标 rpm → 4× 速度 PID → 电流指令 → CAN 0x200
```

**升降 (100Hz)**：

```
拨杆三档 / CH2 微调 → 目标位置(mm → counts)
  → 位置外环 PID → 速度指令 → 速度内环 PID → 电流指令 → CAN 0x1FF
```

**航向保持 (CH10 开启时)**：

```
BMI088 陀螺仪积分 yaw → 与参考航向误差 → 航向 PID → Wz 指令
  (与摇杆旋转互斥, 摇杆转动时摇杆优先)
```

---

## 遥控器操作说明

遥控通道解析见 [remote.h](user/bsp/remote/remote.h)，应用映射见 `app_config.h` 与 [chassis.c](user/app/chassis/chassis.c) / [lift.c](user/app/lift/lift.c)。

| 通道 | 开关 | 功能 |
|:----:|:----:|:-----|
| CH1 `right_x` | 右摇杆左右 | 横移 `Vy`（左右平移） |
| CH3 `right_y` | 右摇杆上下 | 前进 / 后退 `Vx` |
| CH4 `left_x` | 左摇杆左右 | 自转 `Wz` |
| CH9 `Se` | 三档拨杆 | 升降目标挡位（上=最高挡，下=最低点） |
| CH2 `left_y` | 左摇杆上下 | 升降微调（配合 CH9 回中进入微调模式，底盘微调期间禁止自转） |
| CH10 `Sg` | 拨杆开关 | 陀螺仪航向保持开/关 |
| CH5 `Sa` | 拨杆 | 电磁阀 1 (PB12) |
| CH7 `Sc` | 拨杆 | 电磁阀 2 (PB13) |
| CH8 `Sd` | 拨杆 | 电磁阀 3 (PB14) |
| CH6 `Sb` | 拨杆 | 电磁阀 4 (PB15) |

> 注：`CH5~CH8` 为高电平有效（通电开阀），若使用 12V/24V 电磁阀需外接 MOS/继电器驱动。

---

## 构建与烧录

固件由 STM32CubeMX 生成工程，使用 CMake + Ninja + arm-none-eabi-gcc（STM32CubeCLT）构建，同时也保留了 MDK-ARM（Keil）工程。

**环境依赖**
- CMake ≥ 3.22、Ninja、arm-none-eabi-gcc（STM32CubeCLT，已加入 PATH）
- VSCode `stmicroelectronics.stm32cube-ide-build-cmake` 扩展（提供 cube-cmake）

**编译**

```bash
./build.sh          # 只编译, 产物在 build/Debug/NatonalCarSet.elf
```

**编译并烧录**（OpenOCD 0.12 + CMSIS-DAP）

```bash
./flash.sh          # 编译 → 烧录 → 校验 → 复位运行
```

或在 VSCode 中使用 CMakePresets：`Debug` / `Release` / `编译`。

---

## 参数配置

所有可调参数集中在 [user/app/app_config/app_config.h](user/app/app_config/app_config.h)，是整车的"参数单页"：

| 分区 | 内容 |
|:-----|:-----|
| §1 系统控制频率 | 底盘 200Hz / 升降 100Hz / 监控 10Hz |
| §2 CAN 通信 | 波特率、控制/反馈 ID、超时时间 |
| §3 底盘机械 | 轮距/轴距/轮径、满杆转速、加减速限幅、死区 |
| §4 升降机构 | 减速比、导程（迭代标定值）、软限位、预设挡位、微调曲线 |
| §5 PID 参数 | 底盘速度环 ×4、升降串级、航向保持 |
| §6 电磁阀 | GPIO 引脚与遥控通道映射 |
| §7 LED 指示 | 状态灯引脚 |
| §8 电机索引 | 物理位置 ↔ 代码索引 ↔ CAN ID 对应 |
| §9 任务配置 | 优先级与栈大小 |
| §11 陀螺仪 | BMI088 引脚、灵敏度、航向 PID |
| §12 WS2815 | 灯珠数量、呼吸灯参数 |

> 带 `TODO` 标注的参数（如机械尺寸、满杆转速、PID 增益）需在实车标定。修改后重新编译即可生效，无需改动业务代码。

---

## 故障诊断与状态指示

### PE1 状态 LED

| 现象 | 含义 |
|:-----|:-----|
| 常亮 | 系统正常 |
| 500ms 周期闪烁 | 遥控器信号丢失 |
| 200ms 周期闪烁 | 电机 CAN 通信超时 |
| 熄灭 | 急停 (ESTOP) |

### 故障码（"死机"定位）

异常触发时由 `Fault_Record()` 记录故障码到 RAM + RTC 备份寄存器（跨复位保留）：

- 复位后上电：`Watchdog_Init` 会以 LED 闪烁 `(code+1)` 次提示上次故障类型
- 调试器 Watch 窗口可直接查看 `g_fault_code`

| 码 | 含义 | 码 | 含义 |
|:--:|:-----|:--:|:-----|
| 0 | 无故障 | 5 | UsageFault |
| 1 | 任务栈溢出 | 6 | configASSERT 触发 |
| 2 | HardFault | 7 | FreeRTOS 堆分配失败 |
| 3 | MemManage | 8 | HAL Error_Handler |
| 4 | BusFault | | |

---

## 目录结构

```
NatonalCarSet/
├── Core/                    # CubeMX 生成: 启动、时钟、外设初始化
│   ├── Inc/  Src/
├── Drivers/                 # ST 官方 HAL / CMSIS 库
├── Middlewares/             # FreeRTOS 中间件
├── user/                    # 用户代码 (业务全部在此)
│   ├── algo/
│   │   ├── pid/             # PID 控制器 (积分分离 + 微分滤波)
│   │   └── vofa/            # VOFA+ 串口波形协议
│   ├── app/
│   │   ├── app_config/      # ★ 全局参数单页 (集中配置)
│   │   ├── chassis/         # 麦克纳姆轮底盘 (运动学 + 4路速度PID + 航向保持)
│   │   ├── lift/            # 升降机构 (串级PID)
│   │   └── watchdog/        # 安全监控 (遥控/CAN/任务卡死 + 故障记录)
│   └── bsp/
│       ├── imu/             # BMI088 驱动 + 航向积分
│       ├── motor/           # M3508 CAN 通信层
│       ├── remote/          # SBUS 遥控解析
│       ├── valve/           # 电磁阀控制
│       └── ws2815/          # WS2815 双灯带 (PWM+DMA)
├── docs/                    # 设计文档
├── build.sh / flash.sh      # 编译 / 烧录脚本
├── CMakeLists.txt           # 构建入口 (含用户源码/头文件列表)
├── CMakePresets.json
└── NatonalCarSet.ioc        # STM32CubeMX 工程
```

---

## 相关文档

- [电机 CAN ID 与物理位置对应关系](docs/DESIGN_MOTOR_CAN_MAPPING.md) — M3508/C620 的 ID 分配、数据帧格式、麦轮运动学、排查清单

---

*项目持续迭代中，控制参数以实车标定为准。*
