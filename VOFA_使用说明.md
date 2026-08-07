# VOFA+ 调试遥测 — 使用说明

> **适用项目**: NationalCar (M3508 麦科勒姆轮底盘)  
> **文件位置**: `NatonalCarSet/user/algo/vofa/vofa.h` / `vofa.c`  
> **日期**: 2026-07-26

---

## 1. 概述

VOFA+ 是一款串口数据可视化调试软件，支持多种通信协议。项目中通过 USART6 将电机转速等实时数据以 **JustFloat** 协议发送到 PC 端的 VOFA+ 软件，实现波形图的实时显示和调试。

```
┌──────────┐   USART6 (115200/8N1)   ┌───────────┐   USB   ┌──────────┐
│ STM32F407│──────────────────────▶  │ USB-TTL   │───────▶│  VOFA+   │
│  PG14 TX │   TX ──────────── RX    │ 模块      │        │  软件    │
│  GND     │   GND ─────────── GND   │(CH340/CP) │        │  波形图   │
└──────────┘                         └───────────┘        └──────────┘
```

---

## 2. 硬件连接

| STM32 引脚 | 功能 | USB-TTL 模块 |
|-----------|------|-------------|
| **PG14** | USART6_TX (AF8) | RX |
| **GND** | 共地 | GND |

> **注意**: 只需接 TX 和 GND 两根线。不需要接 RX，因为数据是 STM32 → PC 单向发送。

USART6 已在 `NatonalCarSet.ioc` (CubeMX) 中配置：

| 参数 | 值 |
|------|-----|
| 波特率 | 115200 bps |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | None |
| 模式 | Asynchronous |

---

## 3. JustFloat 协议详解

### 3.1 帧格式

```
┌──────────┬──────────┬──────────┬──────────┬──────────┐
│ Float 0  │ Float 1  │  0x00    │  0x00    │  0x80 0x7F│
│ (4 bytes)│ (4 bytes)│          │          │ (小端)    │
└──────────┴──────────┴──────────┴──────────┴──────────┘
  Channel 1  Channel 2  ←──────── 帧尾 (4 bytes) ──────→

总长度: 12 bytes (2 个 float × 4 + 4 字节帧尾)
```

- 帧尾固定为 `0x00 0x00 0x80 0x7F`，VOFA+ 用它识别帧边界和字节序（小端/大端）。
- 每个 `float` 为 IEEE 754 单精度浮点数（4 字节），STM32 和 PC 默认均为小端序，无需转换。
- 通道数理论上可扩展：`N*4 + 4` 字节，当前代码只发了 **2 个通道**。

### 3.2 当前通道定义

| 通道 | 变量名 | 含义 |
|------|--------|------|
| CH1 | `target_speed` | 目标转速（rpm） |
| CH2 | `current_speed` | 实际转速（rpm） |

---

## 4. 代码结构

### 4.1 `vofa.h`

```c
#ifndef VOFA_H
#define VOFA_H

#include "stm32f4xx_hal.h"

void VOFA_JustFloat_Send(float target_speed, float current_speed);

#endif
```

### 4.2 `vofa.c`

```c
#include "vofa.h"
#include <string.h>

extern UART_HandleTypeDef huart6;  // CubeMX 生成的 USART6 句柄

static uint8_t vofa_tx_buf[12];   // 帧缓冲区
static const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};  // 帧尾

void VOFA_JustFloat_Send(float target_speed, float current_speed)
{
    // 1. 非阻塞保护: USART6 忙则跳过本帧
    if (huart6.gState != HAL_UART_STATE_READY) return;

    // 2. 打包 float 数据（小端序，直接 memcpy）
    memcpy(&vofa_tx_buf[0], &target_speed,  sizeof(target_speed));
    memcpy(&vofa_tx_buf[4], &current_speed, sizeof(current_speed));

    // 3. 追加帧尾
    vofa_tx_buf[8]  = tail[0];  // 0x00
    vofa_tx_buf[9]  = tail[1];  // 0x00
    vofa_tx_buf[10] = tail[2];  // 0x80
    vofa_tx_buf[11] = tail[3];  // 0x7F

    // 4. 中断模式发送（非阻塞，不卡任务）
    (void)HAL_UART_Transmit_IT(&huart6, vofa_tx_buf, sizeof(vofa_tx_buf));
}
```

**关键设计要点：**

| 特性 | 说明 |
|------|------|
| **非阻塞发送** | 使用 `HAL_UART_Transmit_IT()` 中断模式，不阻塞调用者（如底盘控制任务） |
| **掉帧保护** | 若 USART6 正在发送上一帧，`gState != READY` 直接跳过，不排队积压 |
| **静态缓冲** | `vofa_tx_buf` 为 `static`，不占用栈空间 |
| **帧尾校验** | `0x00 0x00 0x80 0x7F` 是小端浮点 `+∞` 的 IEEE 754 表示，VOFA+ 以此识别帧边界 |

---

## 5. 在任务中调用

当前 `vofa.c` **尚未被任何任务实际调用**。需要在控制任务中加入调用代码。以下是推荐的调用位置：

### 5.1 底盘控制任务 (`freertos.c` → `StartTaskChassis`)

```c
void StartTaskChassis(void *argument)
{
    (void)argument;
    TickType_t last_wake = osKernelGetTickCount();

    for (;;) {
        Chassis_Update();

        // ★ 发送 VOFA+ 遥测（以电机 0 为例）
        VOFA_JustFloat_Send(
            g_chassis.wheel_rpm_target[0],    // CH1: 目标转速
            (float)g_motor.feedback[0].speed_rpm  // CH2: 实际转速
        );

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CHASSIS_PERIOD_MS));
    }
}
```

### 5.2 升降控制任务 (`StartTaskLift`)

```c
void StartTaskLift(void *argument)
{
    (void)argument;
    TickType_t last_wake = osKernelGetTickCount();

    for (;;) {
        Lift_Update();

        VOFA_JustFloat_Send(
            g_lift.target_position,      // CH1: 目标位置
            g_lift.current_position      // CH2: 当前位置
        );

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LIFT_PERIOD_MS));
    }
}
```

### 5.3 注意事项

1. **不能在 ISR 中调用** — `HAL_UART_Transmit_IT()` 不是 ISR-safe 的
2. **发送频率不宜过高** — 底盘 200Hz 每周期都发可能导致串口拥塞，建议降频发送（如每 N 个周期发一次），或扩展为 4~8 通道一帧发完
3. **USART6 初始化** — CubeMX 已生成 `MX_USART6_UART_Init()`，在 `main.c` 中调用，无需额外配置

---

## 6. VOFA+ 软件配置

### 6.1 下载与安装

- 官网: [https://www.vofa.plus](https://www.vofa.plus)
- 免安装，解压直接运行 `vofa+.exe`

### 6.2 配置步骤

1. 启动 VOFA+，左侧选择协议 → **JustFloat**
2. 点击左上角 **端口配置**：
   - 端口号: 选择 USB-TTL 对应的 COM 口
   - 波特率: **115200**
   - 数据位: 8 / 停止位: 1 / 校验: None
3. 拖一个 **波形图** 控件到画布
4. 右键波形图 → **设置**：
   - 绑定 **CH1**（目标转速，对应 `target_speed`）
   - 绑定 **CH2**（实际转速，对应 `current_speed`）
   - 可设置颜色、Y轴范围等
5. 点击左上角 **连接** 按钮，开始接收数据

### 6.3 调试界面速查

| 操作 | 方法 |
|------|------|
| 波形图缩放 | 滚轮：时间轴缩放；Ctrl+滚轮：幅值缩放 |
| 拖动波形 | 左键拖动 |
| 暂停/恢复 | 空格键 |
| 保存数据 | 右键 → 导出 CSV |
| 查看数值 | 鼠标悬停在波形上 |

---

## 7. 扩展：多通道版本

当前只有 2 个通道，调试底盘时需要看 4 个电机。以下是扩展方案：

```c
// vofa.h — 多通道版本
#define VOFA_CHANNELS 8

void VOFA_JustFloat_SendMulti(float *data, uint8_t count);

// vofa.c
static uint8_t vofa_tx_buf[VOFA_CHANNELS * 4 + 4];  // N*4 + 帧尾

void VOFA_JustFloat_SendMulti(float *data, uint8_t count)
{
    if (huart6.gState != HAL_UART_STATE_READY) return;
    if (count > VOFA_CHANNELS) count = VOFA_CHANNELS;

    memcpy(vofa_tx_buf, data, count * sizeof(float));
    memcpy(&vofa_tx_buf[count * 4], tail, 4);

    (void)HAL_UART_Transmit_IT(&huart6, vofa_tx_buf, count * 4 + 4);
}
```

调用示例（8 通道：4 个电机 × 目标+实际）：

```c
float vofa_data[8] = {
    g_chassis.wheel_rpm_target[0], (float)g_motor.feedback[0].speed_rpm,
    g_chassis.wheel_rpm_target[1], (float)g_motor.feedback[1].speed_rpm,
    g_chassis.wheel_rpm_target[2], (float)g_motor.feedback[2].speed_rpm,
    g_chassis.wheel_rpm_target[3], (float)g_motor.feedback[3].speed_rpm,
};
VOFA_JustFloat_SendMulti(vofa_data, 8);
```

VOFA+ 侧会自动识别 8 个通道，波形图绑定 CH1~CH8 即可。

---

## 8. 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 无数据显示 | COM 口选错 / 波特率不匹配 | 检查设备管理器 COM 口，确认 115200 |
| 波形乱跳 | 帧尾丢失导致帧错位 | 检查 GND 是否共地，尝试下拉框手动切字节序 |
| 数据明显滞后 | 发送频率过高，串口 115200 带宽不够 | 降低发送频率（隔周期发），或提高波特率至 921600 |
| 编译报错 `huart6` 未声明 | CubeMX 没生成 USART6 | 在 CubeMX 中使能 USART6，重新生成代码 |
| 发送中断卡死 | USART6 中断未使能 | 在 CubeMX NVIC 中使能 USART6 global interrupt |

---

## 9. 相关文件索引

| 文件 | 说明 |
|------|------|
| `NatonalCarSet/user/algo/vofa/vofa.h` | VOFA+ 发送函数声明 |
| `NatonalCarSet/user/algo/vofa/vofa.c` | JustFloat 协议实现 |
| `NatonalCarSet/Core/Src/usart.c` | CubeMX 生成 USART6 初始化 |
| `NatonalCarSet/Core/Src/main.c` | `MX_USART6_UART_Init()` 调用位置 |
| `NatonalCarSet/Core/Src/freertos.c` | 控制任务 → 调用 VOFA_Send |
| `NatonalCarSet/user/app/chassis/chassis.c` | 底盘控制 → 提供转速数据 |
| `NatonalCarSet/user/app/lift/lift.c` | 升降控制 → 提供位置数据 |
