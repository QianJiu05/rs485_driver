#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_modbus.h"
#include "pid.h"

/* 通信超时默认值(ms) */
#ifndef MOTOR_CTRL_MODBUS_TIMEOUT
#define MOTOR_CTRL_MODBUS_TIMEOUT       (300U)
#endif

/* PID输出对应的最大电流，单位 10mA（默认±50A） */
#ifndef MOTOR_CTRL_MAX_CURRENT_10MA
#define MOTOR_CTRL_MAX_CURRENT_10MA     (5000)
#endif

/**
 * @brief 电机控制器运行状态
 */
typedef enum
{
    MOTOR_CTRL_STOPPED = 0,     /* 停止 */
    MOTOR_CTRL_RUNNING = 1      /* 运行中 */
} motor_ctrl_state_t;

/**
 * @brief 电机控制器实例（速度闭环 → 电流输出）
 */
typedef struct
{
    pid_obj_t *speed_pid;           /* 速度环PID实例 */
    motor_ctrl_state_t state;       /* 当前运行状态 */
    int32_t target_rpm;             /* 目标转速(rpm) */
    int32_t actual_rpm;             /* 实时转速(rpm) */
    int16_t output_current_10ma;    /* PID输出电流(10mA) */
    uint16_t fault_code;            /* 最近一次故障码 */
    uint32_t comm_error_count;      /* 通信连续失败计数 */
} motor_ctrl_t;

/**
 * @brief 初始化电机控制器，创建PID实例并初始化底层Modbus通信。
 * @param pid_cfg 速度环PID配置参数指针。
 */
void motor_ctrl_init(pid_config_t *pid_cfg);

/**
 * @brief 启动电机控制（切入电流模式，开始闭环）。
 */
void motor_ctrl_start(void);

/**
 * @brief 停止电机控制（写零电流，清空PID历史）。
 */
void motor_ctrl_stop(void);

/**
 * @brief 设置目标转速。
 * @param rpm 目标转速，单位 rpm，正值正转，负值反转。
 */
void motor_ctrl_set_target_rpm(int32_t rpm);

/**
 * @brief 控制循环周期函数，应在主循环或定时中断中周期调用。
 *        完成：心跳更新 → 读取实时转速 → PID计算 → 写入目标电流。
 */
void motor_ctrl_loop(void);

/**
 * @brief 获取电机控制器实例的只读指针，可用于外部监测状态。
 * @return 电机控制器实例指针。
 */
const motor_ctrl_t *motor_ctrl_get_instance(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H__ */
