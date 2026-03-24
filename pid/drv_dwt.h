/**
 * @file drv_dwt.h
 * @brief 基于Cortex-M4 DWT(数据观察点与跟踪)模块的高精度计时驱动
 */

#ifndef __DRV_DWT_H__
#define __DRV_DWT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief 初始化DWT计数器，使能CYCCNT周期计数
 */
void dwt_init(void);

/**
 * @brief 获取当前DWT周期计数值
 * @return 当前CYCCNT值
 */
uint32_t dwt_get_tick(void);

/**
 * @brief 计算距上次调用的时间间隔（秒），并更新计数快照
 * @param cnt_last 上次保存的CYCCNT快照指针，调用后会被更新为当前值
 * @return 两次调用之间的时间间隔，单位秒(float)
 */
float dwt_get_delta(uint32_t *cnt_last);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_DWT_H__ */
