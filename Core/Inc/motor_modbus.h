#ifndef __MOTOR_MODBUS_H__
#define __MOTOR_MODBUS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "modbus_rtu.h"
#include "motor_registers.h"

/**
 * @brief 驱动器 ModBus 从站地址（可按需修改）
 */
#ifndef MOTOR_MODBUS_SLAVE_ID
#define MOTOR_MODBUS_SLAVE_ID                  (1U)
#endif

/**
 * @brief 电机磁极对数（rpm 转 erpm 使用）
 */
#ifndef MOTOR_MODBUS_POLE_PAIRS
#define MOTOR_MODBUS_POLE_PAIRS                (4U)
#endif

#define MOTOR_MODBUS_REG_HEARTBEAT             HEARTBEAT_REG
#define MOTOR_MODBUS_REG_CONTROL_MODE          CONTROL_MODE_REG
#define MOTOR_MODBUS_REG_TARGET_CURRENT        TARGET_CURRENT_REG
#define MOTOR_MODBUS_REG_TARGET_SPEED_H        TARGET_SPEED_REG_H
#define MOTOR_MODBUS_REG_SPEED_LOOP_ACCEL_H    SPEED_LOOP_ACCEL_REG_H
#define MOTOR_MODBUS_REG_SPEED_LOOP_DECEL_H    SPEED_LOOP_DECEL_REG_H

#define MOTOR_MODBUS_REG_FAULT_INFO            FAULT_INFO_REG
#define MOTOR_MODBUS_REG_REALTIME_SPEED_H      REALTIME_SPEED_REG_H

#define MOTOR_MODBUS_MODE_CURRENT_CONTROL      (0U)
#define MOTOR_MODBUS_MODE_SPEED_CONTROL        (1U)

void motor_modbus_init(void);

modbus_status_t motor_modbus_write_heartbeat(uint16_t heartbeat_value,
                                             uint32_t timeout_ms);

modbus_status_t motor_modbus_heartbeat_tick(uint32_t timeout_ms);

modbus_status_t motor_modbus_set_current_mode(uint32_t timeout_ms);

modbus_status_t motor_modbus_set_target_current_10ma(int16_t current_10ma,
                                                     uint32_t timeout_ms);

modbus_status_t motor_modbus_set_target_current_ma(int32_t current_ma,
                                                   uint32_t timeout_ms);

modbus_status_t motor_modbus_set_current_control_10ma(int16_t current_10ma,
                                                      uint32_t timeout_ms);

modbus_status_t motor_modbus_set_speed_mode(uint32_t timeout_ms);

modbus_status_t motor_modbus_set_target_speed_erpm(int32_t speed_erpm,
                                                   uint32_t timeout_ms);

modbus_status_t motor_modbus_set_speed_loop_accel_erpm_s(int32_t accel_erpm_s,
                                                         uint32_t timeout_ms);

modbus_status_t motor_modbus_set_speed_loop_decel_erpm_s(int32_t decel_erpm_s,
                                                         uint32_t timeout_ms);

modbus_status_t motor_modbus_set_speed_control_erpm(int32_t speed_erpm,
                                                    uint32_t timeout_ms);

modbus_status_t motor_modbus_set_target_speed_rpm(int32_t speed_rpm,
                                                  uint16_t pole_pairs,
                                                  uint32_t timeout_ms);

modbus_status_t motor_modbus_set_speed_control_rpm(int32_t speed_rpm,
                                                   uint16_t pole_pairs,
                                                   uint32_t timeout_ms);

/**
 * @brief 读取故障信息寄存器(5000)。
 * @param fault_info 输出: 故障码。
 * @param timeout_ms 通信超时时间。
 */
modbus_status_t motor_modbus_read_fault_info(uint16_t *fault_info,
                                             uint32_t timeout_ms);

/**
 * @brief 读取实时转速(5001-5002), 单位 erpm。
 * @param speed_erpm 输出: 实时转速(erpm)。
 * @param timeout_ms 通信超时时间。
 */
modbus_status_t motor_modbus_read_speed_erpm(int32_t *speed_erpm,
                                             uint32_t timeout_ms);

/**
 * @brief 调试用: 纯发送目标电流帧, 不等待响应。
 */
modbus_status_t motor_modbus_debug_send_current_10ma(int16_t current_10ma,
                                                     uint32_t timeout_ms);

/**
 * @brief 调试用: 纯发送控制模式帧, 不等待响应。
 */
modbus_status_t motor_modbus_debug_send_current_mode(uint32_t timeout_ms);

/**
 * @brief 调试用: 纯发送心跳帧, 不等待响应。
 */
modbus_status_t motor_modbus_debug_send_heartbeat(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_MODBUS_H__ */
