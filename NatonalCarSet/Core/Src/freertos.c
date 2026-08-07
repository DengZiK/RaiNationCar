/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ws2812.h"
#include "app_config.h"
#include "watchdog.h"
#include "remote.h"
#include "chassis.h"
#include "lift.h"
#include "valve.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* Definitions for WS2812_task — 已注释 (灯带 DMA 忙等 osDelay(0) 可能饿死低优先级任务) */
/* osThreadId_t taskWs2812Handle; */
/* const osThreadAttr_t taskWs2812_attributes = { */
/*   .name = "WS2812_task", */
/*   .stack_size = 256 * 4, */
/*   .priority = (osPriority_t) osPriorityNormal, */
/* }; */

/* Definitions for Watchdog_task */
osThreadId_t taskWatchdogHandle;
const osThreadAttr_t taskWatchdog_attributes = {
  .name = "Watchdog_task",
  .stack_size = STACK_WATCHDOG,
  .priority = (osPriority_t) PRIO_WATCHDOG,
};

/* Definitions for Remote_task */
osThreadId_t taskRemoteHandle;
const osThreadAttr_t taskRemote_attributes = {
  .name = "Remote_task",
  .stack_size = STACK_REMOTE,
  .priority = (osPriority_t) PRIO_REMOTE,
};

/* Definitions for Chassis_task */
osThreadId_t taskChassisHandle;
const osThreadAttr_t taskChassis_attributes = {
  .name = "Chassis_task",
  .stack_size = STACK_CHASSIS,
  .priority = (osPriority_t) PRIO_CHASSIS,
};

/* Definitions for Lift_task */
osThreadId_t taskLiftHandle;
const osThreadAttr_t taskLift_attributes = {
  .name = "Lift_task",
  .stack_size = STACK_LIFT,
  .priority = (osPriority_t) PRIO_LIFT,
};

/* Definitions for Valve_task */
osThreadId_t taskValveHandle;
const osThreadAttr_t taskValve_attributes = {
  .name = "Valve_task",
  .stack_size = STACK_VALVE,
  .priority = (osPriority_t) PRIO_VALVE,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Watchdog_task(void *argument);
void Remote_task(void *argument);
void Chassis_task(void *argument);
void Lift_task(void *argument);
void Valve_task(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* taskWs2812Handle  = osThreadNew(WS2812_task, NULL, &taskWs2812_attributes);  // 灯带任务已注释 */
  taskWatchdogHandle = osThreadNew(Watchdog_task, NULL, &taskWatchdog_attributes);
  taskRemoteHandle   = osThreadNew(Remote_task, NULL, &taskRemote_attributes);
  taskChassisHandle  = osThreadNew(Chassis_task, NULL, &taskChassis_attributes);
  taskLiftHandle     = osThreadNew(Lift_task, NULL, &taskLift_attributes);
  taskValveHandle    = osThreadNew(Valve_task, NULL, &taskValve_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* ========================================================================= */
/*  Watchdog 安全监控任务 — 最高优先级 (10Hz / 100ms)                          */
/*                                                                            */
/*  职责:                                                                      */
/*    1. 遥控器连接检查 + 电机 CAN 超时检查                                     */
/*    2. LED 状态指示                                                           */
/*    3. 喂硬件 IWDG (任务卡死 ~1s 自动复位)                                    */
/* ========================================================================= */
void Watchdog_task(void *argument)
{
    (void)argument;
    Watchdog_StartIWDG();   /* 启动硬件看门狗，一旦启动不可关闭 */

    const TickType_t period = pdMS_TO_TICKS(100);  /* 10Hz */
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        Watchdog_Update();
        vTaskDelayUntil(&wake, period);
    }
}

/* ========================================================================= */
/*  Remote 遥控器看门狗任务 — 次高优先级 (50Hz / 20ms)                          */
/*                                                                            */
/*  遥控器帧是异步 ISR 处理 (SBUS IDLE 中断), 本任务只负责超时检测。           */
/*  50Hz 空闲循环足够感知 100ms 超时。                                          */
/* ========================================================================= */
void Remote_task(void *argument)
{
    (void)argument;

    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        remote_control_watchdog();   /* 快速返回, 仅检查 last_frame_tick */
        vTaskDelayUntil(&wake, period);
    }
}

/* ========================================================================= */
/*  Chassis 底盘控制任务 — 正常优先级 (200Hz / 5ms)                             */
/*                                                                            */
/*  核心控制循环: 摇杆 → 运动学 → 4路速度PID → CAN 电流指令                     */
/*  IMU 航向积分 (IMU_Update) 在本任务内调用以保证 200Hz 积分频率。             */
/* ========================================================================= */
void Chassis_task(void *argument)
{
    (void)argument;

    const TickType_t period = pdMS_TO_TICKS(CHASSIS_PERIOD_MS);
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        Chassis_Update();
        vTaskDelayUntil(&wake, period);
    }
}

/* ========================================================================= */
/*  Lift 升降控制任务 — 正常优先级 (100Hz / 10ms)                               */
/*                                                                            */
/*  串级 PID: 位置外环 → 速度内环 → CAN 电流指令                                */
/* ========================================================================= */
void Lift_task(void *argument)
{
    (void)argument;

    const TickType_t period = pdMS_TO_TICKS(LIFT_PERIOD_MS);
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        Lift_Update();
        vTaskDelayUntil(&wake, period);
    }
}

/* ========================================================================= */
/*  Valve 电磁阀任务 — 最低优先级 (50Hz / 20ms)                                  */
/*                                                                            */
/*  4 路拨杆边沿检测 → GPIO 开关, 低速不抢 CPU                                  */
/* ========================================================================= */
void Valve_task(void *argument)
{
    (void)argument;

    const TickType_t period = pdMS_TO_TICKS(20);
    TickType_t wake = xTaskGetTickCount();

    while (1) {
        Valve_Process();
        vTaskDelayUntil(&wake, period);
    }
}

/* USER CODE END Application */

