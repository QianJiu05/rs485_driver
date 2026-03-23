#ifndef __MOTOR_REGISTERS_H__
#define __MOTOR_REGISTERS_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 输入寄存器(只读)地址映射
 */
#define FAULT_INFO_REG                         (5000U) /* 故障信息 */
#define REALTIME_SPEED_REG_H                   (5001U) /* 实时转速高16位(erpm) */
#define REALTIME_SPEED_REG_L                   (5002U) /* 实时转速低16位(erpm) */
#define REALTIME_DUTY_REG                      (5003U) /* 实时占空比(-1000~1000) */
#define REALTIME_POWER_REG                     (5004U) /* 实时功率(W) */
#define REALTIME_INPUT_VOLTAGE_REG             (5005U) /* 实时输入电压(V) */
#define REALTIME_MOTOR_CURRENT_REG             (5006U) /* 实时电机电流(10mA) */
#define REALTIME_BUS_CURRENT_REG               (5007U) /* 实时总线电流(10mA) */
#define REALTIME_TEMPERATURE_REG               (5008U) /* 实时温度(℃) */
#define REALTIME_ANGLE_REG                     (5009U) /* 实时角度(0.01度) */
#define REALTIME_POSITION_REG_H                (5010U) /* 实时位置高16位(0.01度累加) */
#define REALTIME_POSITION_REG_L                (5011U) /* 实时位置低16位(0.01度累加) */
#define HOMING_STATUS_REG                      (5012U) /* 回零状态(高8位状态/低8位代码) */
#define Z_SIGNAL_FOUND_REG                     (5013U) /* Z信号检测状态(1找到/0未找到) */
#define HISTORY_ERROR_1_REG                    (5014U) /* 历史错误1(最近错误) */
#define HISTORY_ERROR_2_REG                    (5015U) /* 历史错误2 */
#define HISTORY_ERROR_3_REG                    (5016U) /* 历史错误3 */
#define HISTORY_ERROR_4_REG                    (5017U) /* 历史错误4 */
#define HISTORY_ERROR_5_REG                    (5018U) /* 历史错误5 */
#define INPUT_IO_STATUS_REG                    (5019U) /* 输入IO状态(低8位位图) */
#define OUTPUT_IO_STATUS_REG                   (5020U) /* 输出IO状态(低8位位图) */

/**
 * @brief 保持寄存器(读写)地址映射
 */
#define HEARTBEAT_REG                          (6000U) /* 心跳寄存器 */
#define CONTROL_MODE_REG                       (6001U) /* 控制模式 */
#define TARGET_CURRENT_REG                     (6002U) /* 设定电流(10mA) */
#define TARGET_SPEED_REG_H                     (6003U) /* 设定转速高16位(erpm) */
#define TARGET_SPEED_REG_L                     (6004U) /* 设定转速低16位(erpm) */
#define TARGET_DUTY_REG                        (6005U) /* 设定占空比(-1000~1000) */
#define TARGET_ABS_POSITION_REG_H              (6006U) /* 设定绝对位置高16位(0.01度) */
#define TARGET_ABS_POSITION_REG_L              (6007U) /* 设定绝对位置低16位(0.01度) */
#define TARGET_REL_LAST_REG_H                  (6008U) /* 相对上次目标位置高16位(0.01度) */
#define TARGET_REL_LAST_REG_L                  (6009U) /* 相对上次目标位置低16位(0.01度) */
#define TARGET_REL_CURRENT_REG_H               (6010U) /* 相对当前位置高16位(0.01度) */
#define TARGET_REL_CURRENT_REG_L               (6011U) /* 相对当前位置低16位(0.01度) */
#define SET_CURRENT_POSITION_REG_H             (6012U) /* 设置当前位置高16位(0.01度) */
#define SET_CURRENT_POSITION_REG_L             (6013U) /* 设置当前位置低16位(0.01度) */
#define BRAKE_CURRENT_REG                      (6014U) /* 设定刹车电流(10mA) */
#define HANDBRAKE_CURRENT_REG                  (6015U) /* 设定手刹电流(10mA) */
#define SPEED_LOOP_ACCEL_REG_H                 (6016U) /* 速度环加速度高16位(erpm/s) */
#define SPEED_LOOP_ACCEL_REG_L                 (6017U) /* 速度环加速度低16位(erpm/s) */
#define TRAJECTORY_MAX_SPEED_REG_H             (6018U) /* 轨迹最大速度高16位(erpm) */
#define TRAJECTORY_MAX_SPEED_REG_L             (6019U) /* 轨迹最大速度低16位(erpm) */
#define TRAJECTORY_MAX_ACCEL_REG_H             (6020U) /* 轨迹最大加速度高16位(erpm/min) */
#define TRAJECTORY_MAX_ACCEL_REG_L             (6021U) /* 轨迹最大加速度低16位(erpm/min) */
#define TRAJECTORY_MAX_DECEL_REG_H             (6022U) /* 轨迹最大减速度高16位(erpm/min) */
#define TRAJECTORY_MAX_DECEL_REG_L             (6023U) /* 轨迹最大减速度低16位(erpm/min) */
#define ACTIVE_MOTOR_PROFILE_REG               (6024U) /* 当前生效电机配置表 */
#define SPEED_LOOP_DECEL_REG_H                 (6025U) /* 速度环减速度高16位(erpm/s) */
#define SPEED_LOOP_DECEL_REG_L                 (6026U) /* 速度环减速度低16位(erpm/s) */
#define HOMING_MODE_REG                        (6027U) /* 回零模式 */
#define CLOSED_LOOP_MAX_CURRENT_REG            (6028U) /* 闭环模式最大电流 */
#define CURRENT_RAMP_ACCEL_REG                 (6029U) /* 电流爬升加速度 */
#define CURRENT_RAMP_TARGET_REG                (6030U) /* 电流爬升目标电流 */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_REGISTERS_H__ */
