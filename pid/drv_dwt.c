/**
 * @file drv_dwt.c
 * @brief 基于Cortex-M4 DWT(数据观察点与跟踪)模块的高精度计时驱动实现
 */

#include "drv_dwt.h"

/* CPU主频缓存，用于将周期数换算为秒 */
static uint32_t cpu_freq_hz = 0U;

/**
 * @brief 初始化DWT计数器，使能CYCCNT周期计数。
 *        必须在使用 dwt_get_delta 前调用一次。
 */
void dwt_init(void)
{
    cpu_freq_hz = HAL_RCC_GetSysClockFreq();

    /* 使能DWT模块（通过CoreDebug DEMCR寄存器） */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* 清零并使能周期计数器 */
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 获取当前DWT周期计数值。
 * @return 当前CYCCNT值。
 */
uint32_t dwt_get_tick(void)
{
    return DWT->CYCCNT;
}

/**
 * @brief 计算距上次调用的时间间隔（秒），并更新计数快照。
 * @param cnt_last 上次保存的CYCCNT快照指针，调用后会被更新为当前值。
 * @return 两次调用之间的时间间隔，单位秒(float)。
 */
float dwt_get_delta(uint32_t *cnt_last)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t delta = now - *cnt_last; /* 无符号减法，自动处理溢出回绕 */

    *cnt_last = now;

    return (float)delta / (float)cpu_freq_hz;
}
