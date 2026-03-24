#include "motor_control.h"
#include "drv_dwt.h"

/* 电机控制器单例 */
static motor_ctrl_t motor_ctrl = {0};

/**
 * @brief 将float值限幅到[-limit, +limit]范围并转为int16_t。
 * @param value 待限幅的浮点值。
 * @param limit 限幅绝对值上限。
 * @return 限幅后的int16_t值。
 */
static int16_t clamp_to_i16(float value, float limit)
{
    if (value > limit)
    {
        value = limit;
    }
    if (value < -limit)
    {
        value = -limit;
    }
    return (int16_t)value;
}

/**
 * @brief 初始化电机控制器，创建PID实例并初始化底层Modbus通信。
 * @param pid_cfg 速度环PID配置参数指针。
 */
void motor_ctrl_init(pid_config_t *pid_cfg)
{
    /* 初始化DWT高精度计时 */
    dwt_init();

    /* 初始化底层Modbus通信 */
    motor_modbus_init();

    /* 创建速度环PID实例 */
    motor_ctrl.speed_pid = pid_register(pid_cfg);

    /* 初始状态为停止 */
    motor_ctrl.state = MOTOR_CTRL_STOPPED;
    motor_ctrl.target_rpm = 0;
    motor_ctrl.actual_rpm = 0;
    motor_ctrl.output_current_10ma = 0;
    motor_ctrl.fault_code = 0U;
    motor_ctrl.comm_error_count = 0U;
}

/**
 * @brief 启动电机控制（切入电流模式，开始闭环）。
 */
void motor_ctrl_start(void)
{
    /* 切换驱动器到电流控制模式 */
    motor_modbus_set_current_mode(MOTOR_CTRL_MODBUS_TIMEOUT);

    /* 清空PID历史，从零开始累积 */
    pid_clear(motor_ctrl.speed_pid);

    motor_ctrl.output_current_10ma = 0;
    motor_ctrl.comm_error_count = 0U;
    motor_ctrl.state = MOTOR_CTRL_RUNNING;
}

/**
 * @brief 停止电机控制（写零电流，清空PID历史）。
 */
void motor_ctrl_stop(void)
{
    motor_ctrl.state = MOTOR_CTRL_STOPPED;
    motor_ctrl.target_rpm = 0;
    motor_ctrl.output_current_10ma = 0;

    /* 写零电流停机 */
    motor_modbus_set_target_current_10ma(0, MOTOR_CTRL_MODBUS_TIMEOUT);

    /* 清空PID历史 */
    pid_clear(motor_ctrl.speed_pid);
}

/**
 * @brief 设置目标转速。
 * @param rpm 目标转速，单位 rpm，正值正转，负值反转。
 */
void motor_ctrl_set_target_rpm(int32_t rpm)
{
    motor_ctrl.target_rpm = rpm;
}

/**
 * @brief 控制循环周期函数，应在主循环或定时中断中周期调用。
 *        完成：心跳更新 → 读取实时转速 → PID计算 → 写入目标电流。
 */
void motor_ctrl_loop(void)
{
    modbus_status_t status;
    int32_t speed_rpm = 0;
    float pid_output = 0.0f;

    /* 步骤1: 更新心跳，维持驱动器通信 */
    status = motor_modbus_heartbeat_tick(MOTOR_CTRL_MODBUS_TIMEOUT);
    if (status != MODBUS_OK)
    {
        motor_ctrl.comm_error_count++;
        return;
    }

    /* 步骤2: 读取实时转速(rpm) */
    status = motor_modbus_read_speed_rpm(&speed_rpm,
                                         MOTOR_MODBUS_POLE_PAIRS,
                                         MOTOR_CTRL_MODBUS_TIMEOUT);
    if (status != MODBUS_OK)
    {
        motor_ctrl.comm_error_count++;
        return;
    }

    motor_ctrl.actual_rpm = speed_rpm;
    motor_ctrl.comm_error_count = 0U;

    /* 非运行状态不执行PID计算 */
    if (motor_ctrl.state != MOTOR_CTRL_RUNNING)
    {
        return;
    }

    /* 步骤3: PID计算，测量值=实时转速，设定值=目标转速，输出=电流(10mA) */
    pid_output = pid_calculate(motor_ctrl.speed_pid,
                               (float)motor_ctrl.actual_rpm,
                               (float)motor_ctrl.target_rpm);

    /* 步骤4: 限幅并写入目标电流 */
    motor_ctrl.output_current_10ma = clamp_to_i16(pid_output,
                                                   (float)MOTOR_CTRL_MAX_CURRENT_10MA);

    motor_modbus_set_target_current_10ma(motor_ctrl.output_current_10ma,
                                         MOTOR_CTRL_MODBUS_TIMEOUT);
}

/**
 * @brief 获取电机控制器实例的只读指针，可用于外部监测状态。
 * @return 电机控制器实例指针。
 */
const motor_ctrl_t *motor_ctrl_get_instance(void)
{
    return &motor_ctrl;
}
