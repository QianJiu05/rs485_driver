# PID 库从 RT-Thread 移植到 HAL 库总结

## 日期
2026-03-24

## 变更概述
将 pid 库从 RT-Thread RTOS 依赖改为基于 STM32F4 HAL 库的裸机实现，保持 PID 算法逻辑不变。

## 依赖替换对照

| 原 RT-Thread 依赖 | 替换为 HAL/标准库 |
|---|---|
| `#include <rtthread.h>` | `#include "stm32f4xx_hal.h"` + `<stdlib.h>` + `<string.h>` |
| `rt_malloc()` | `malloc()` (stdlib) |
| `rt_memset()` | `memset()` (string.h) |
| `drv_dwt.h` (RT-Thread 驱动) | 自行实现 `pid/drv_dwt.h` + `pid/drv_dwt.c` |

## 修改文件清单

### pid/drv_dwt.h + pid/drv_dwt.c（新建）
基于 Cortex-M4 DWT（数据观察点与跟踪）模块实现高精度计时：
- `dwt_init()` — 使能 DWT CYCCNT 周期计数器，缓存 CPU 主频
- `dwt_get_tick()` — 读取当前 CYCCNT 值
- `dwt_get_delta(uint32_t *cnt_last)` — 计算两次调用间的时间间隔（秒），自动处理 32 位溢出回绕

### pid/pid.h
- 头文件依赖从 `<rtthread.h>` 替换为 `"stm32f4xx_hal.h"` + `<stdlib.h>` + `<string.h>`
- 所有 `//` 注释改为 `/* */` C 风格注释

### pid/pid.c
- `rt_malloc` → `malloc`，`rt_memset` → `memset`
- 所有 `//` 注释改为 `/* */` C 风格注释
- DWT 驱动调用接口保持不变（`dwt_get_delta`），底层实现已替换

## 使用注意
- 在调用 `pid_register()` 之前，需先调用 `dwt_init()` 初始化 DWT 计时器
- `dwt_init()` 建议在 `main()` 初始化阶段、`SystemClock_Config()` 之后调用
