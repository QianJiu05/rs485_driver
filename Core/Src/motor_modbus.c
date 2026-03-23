#include "motor_modbus.h"

#include <limits.h>

static uint16_t motor_modbus_heartbeat_value = 0U;

/**
 * @brief 写入单个16位寄存器。
 * @param register_addr 目标寄存器地址。
 * @param value 待写入的16位数据。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
static modbus_status_t motor_modbus_write_u16(uint16_t register_addr,
                                              uint16_t value,
                                              uint32_t timeout_ms)
{
  return modbus_rtu_write_single_register((uint8_t)MOTOR_MODBUS_SLAVE_ID,
                                          register_addr,
                                          value,
                                          timeout_ms);
}

/**
 * @brief 写入连续两个16位寄存器，组合表示一个32位有符号值。
 * @param start_register_addr 起始寄存器地址（高16位地址）。
 * @param value 32位有符号数据。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
static modbus_status_t motor_modbus_write_i32(uint16_t start_register_addr,
                                              int32_t value,
                                              uint32_t timeout_ms)
{
  uint16_t register_values[2] = {0};

  register_values[0] = (uint16_t)(((uint32_t)value >> 16U) & 0xFFFFU);
  register_values[1] = (uint16_t)((uint32_t)value & 0xFFFFU);

  return modbus_rtu_write_multiple_registers((uint8_t)MOTOR_MODBUS_SLAVE_ID,
                                             start_register_addr,
                                             register_values,
                                             2U,
                                             timeout_ms);
}

/**
 * @brief 初始化电机ModBus业务层。
 * @param 无。
 * @retval 无。
 */
void motor_modbus_init(void)
{
  modbus_rtu_init();
  motor_modbus_heartbeat_value = 0U;
}

/**
 * @brief 写入心跳寄存器并更新本地心跳缓存值。
 * @param heartbeat_value 心跳寄存器写入值。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_write_heartbeat(uint16_t heartbeat_value,
                                             uint32_t timeout_ms)
{
  motor_modbus_heartbeat_value = heartbeat_value;
  return motor_modbus_write_u16(MOTOR_MODBUS_REG_HEARTBEAT,
                                heartbeat_value,
                                timeout_ms);
}

/**
 * @brief 心跳值自增并写入心跳寄存器。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_heartbeat_tick(uint32_t timeout_ms)
{
  motor_modbus_heartbeat_value++;
  return motor_modbus_write_heartbeat(motor_modbus_heartbeat_value, timeout_ms);
}

/**
 * @brief 将控制模式设置为电流控制模式。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_current_mode(uint32_t timeout_ms)
{
  return motor_modbus_write_u16(MOTOR_MODBUS_REG_CONTROL_MODE,
                                MOTOR_MODBUS_MODE_CURRENT_CONTROL,
                                timeout_ms);
}

/**
 * @brief 设置目标电流，单位为10mA。
 * @param current_10ma 目标电流值，单位10mA，支持正负值。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_target_current_10ma(int16_t current_10ma,
                                                     uint32_t timeout_ms)
{
  return motor_modbus_write_u16(MOTOR_MODBUS_REG_TARGET_CURRENT,
                                (uint16_t)current_10ma,
                                timeout_ms);
}

/**
 * @brief 设置目标电流，单位为mA，内部自动换算为10mA。
 * @param current_ma 目标电流值，单位mA，支持正负值。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval MODBUS_ERR_PARAM 输入参数超出 int16_t 对应10mA范围。
 * @retval 其他值 通信失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_target_current_ma(int32_t current_ma,
                                                   uint32_t timeout_ms)
{
  int32_t current_10ma = 0;

  if ((current_ma < ((int32_t)INT16_MIN * 10)) ||
      (current_ma > ((int32_t)INT16_MAX * 10)))
  {
    return MODBUS_ERR_PARAM;
  }

  current_10ma = current_ma / 10;
  return motor_modbus_set_target_current_10ma((int16_t)current_10ma, timeout_ms);
}

/**
 * @brief 执行电流控制流程: 先设置目标电流，再切换到电流模式。
 * @param current_10ma 目标电流值，单位10mA，支持正负值。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 全流程执行成功。
 * @retval 其他值 任一步失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_current_control_10ma(int16_t current_10ma,
                                                      uint32_t timeout_ms)
{
  modbus_status_t status;

  /**
   * 参考驱动文档流程: 先写目标电流, 再写控制模式=电流模式。
   */
  status = motor_modbus_set_target_current_10ma(current_10ma, timeout_ms);
  if (status != MODBUS_OK)
  {
    return status;
  }

  return motor_modbus_set_current_mode(timeout_ms);
}

/**
 * @brief 将控制模式设置为转速控制模式。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_speed_mode(uint32_t timeout_ms)
{
  return motor_modbus_write_u16(MOTOR_MODBUS_REG_CONTROL_MODE,
                                MOTOR_MODBUS_MODE_SPEED_CONTROL,
                                timeout_ms);
}

/**
 * @brief 设置目标转速，单位 erpm。
 * @param speed_erpm 目标电角度转速，支持正负值。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_target_speed_erpm(int32_t speed_erpm,
                                                   uint32_t timeout_ms)
{
  return motor_modbus_write_i32(MOTOR_MODBUS_REG_TARGET_SPEED_H,
                                speed_erpm,
                                timeout_ms);
}

/**
 * @brief 设置速度环加速度，单位 erpm/s。
 * @param accel_erpm_s 加速度值，支持正负值。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_speed_loop_accel_erpm_s(int32_t accel_erpm_s,
                                                         uint32_t timeout_ms)
{
  return motor_modbus_write_i32(MOTOR_MODBUS_REG_SPEED_LOOP_ACCEL_H,
                                accel_erpm_s,
                                timeout_ms);
}

/**
 * @brief 设置速度环减速度，单位 erpm/s。
 * @param decel_erpm_s 减速度值，支持正负值。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_speed_loop_decel_erpm_s(int32_t decel_erpm_s,
                                                         uint32_t timeout_ms)
{
  return motor_modbus_write_i32(MOTOR_MODBUS_REG_SPEED_LOOP_DECEL_H,
                                decel_erpm_s,
                                timeout_ms);
}

/**
 * @brief 执行转速控制流程: 先设置目标转速，再切换到转速模式。
 * @param speed_erpm 目标电角度转速，单位 erpm。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 全流程执行成功。
 * @retval 其他值 任一步失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_speed_control_erpm(int32_t speed_erpm,
                                                    uint32_t timeout_ms)
{
  modbus_status_t status;

  status = motor_modbus_set_target_speed_erpm(speed_erpm, timeout_ms);
  if (status != MODBUS_OK)
  {
    return status;
  }

  return motor_modbus_set_speed_mode(timeout_ms);
}

/**
 * @brief 设置目标转速，单位 rpm，内部自动换算为 erpm。
 * @param speed_rpm 目标机械转速，单位 rpm，支持正负值。
 * @param pole_pairs 电机磁极对数，必须大于0。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 写入成功。
 * @retval MODBUS_ERR_PARAM 极对数无效或换算后超出 int32_t 范围。
 * @retval 其他值 写入失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_target_speed_rpm(int32_t speed_rpm,
                                                  uint16_t pole_pairs,
                                                  uint32_t timeout_ms)
{
  int64_t speed_erpm64 = 0;

  if (pole_pairs == 0U)
  {
    return MODBUS_ERR_PARAM;
  }

  speed_erpm64 = (int64_t)speed_rpm * (int64_t)pole_pairs;
  if ((speed_erpm64 > INT32_MAX) || (speed_erpm64 < INT32_MIN))
  {
    return MODBUS_ERR_PARAM;
  }

  return motor_modbus_set_target_speed_erpm((int32_t)speed_erpm64, timeout_ms);
}

/**
 * @brief 执行 rpm 转速控制流程: 先按 rpm 设置目标转速，再切换到转速模式。
 * @param speed_rpm 目标机械转速，单位 rpm，支持正负值。
 * @param pole_pairs 电机磁极对数，必须大于0。
 * @param timeout_ms ModBus通信超时时间，单位毫秒。
 * @retval MODBUS_OK 全流程执行成功。
 * @retval MODBUS_ERR_PARAM 极对数无效或换算后超出 int32_t 范围。
 * @retval 其他值 任一步失败，错误码见 modbus_status_t。
 */
modbus_status_t motor_modbus_set_speed_control_rpm(int32_t speed_rpm,
                                                   uint16_t pole_pairs,
                                                   uint32_t timeout_ms)
{
  modbus_status_t status;

  status = motor_modbus_set_target_speed_rpm(speed_rpm, pole_pairs, timeout_ms);
  if (status != MODBUS_OK)
  {
    return status;
  }

  return motor_modbus_set_speed_mode(timeout_ms);
}

/**
 * @brief 调试用: 纯发送目标电流帧, 不等待从站响应。
 * @param current_10ma 目标电流值，单位10mA。
 * @param timeout_ms UART发送超时时间，单位毫秒。
 * @retval MODBUS_OK 发送成功。
 * @retval 其他值 发送失败。
 */
modbus_status_t motor_modbus_debug_send_current_10ma(int16_t current_10ma,
                                                     uint32_t timeout_ms)
{
  return modbus_rtu_write_single_register_no_resp(
             (uint8_t)MOTOR_MODBUS_SLAVE_ID,
             MOTOR_MODBUS_REG_TARGET_CURRENT,
             (uint16_t)current_10ma,
             timeout_ms);
}

/**
 * @brief 调试用: 纯发送控制模式=电流帧, 不等待从站响应。
 * @param timeout_ms UART发送超时时间，单位毫秒。
 * @retval MODBUS_OK 发送成功。
 * @retval 其他值 发送失败。
 */
modbus_status_t motor_modbus_debug_send_current_mode(uint32_t timeout_ms)
{
  return modbus_rtu_write_single_register_no_resp(
             (uint8_t)MOTOR_MODBUS_SLAVE_ID,
             MOTOR_MODBUS_REG_CONTROL_MODE,
             MOTOR_MODBUS_MODE_CURRENT_CONTROL,
             timeout_ms);
}

/**
 * @brief 调试用: 纯发送心跳帧, 不等待从站响应。
 * @param timeout_ms UART发送超时时间，单位毫秒。
 * @retval MODBUS_OK 发送成功。
 * @retval 其他值 发送失败。
 */
modbus_status_t motor_modbus_debug_send_heartbeat(uint32_t timeout_ms)
{
  motor_modbus_heartbeat_value++;
  return modbus_rtu_write_single_register_no_resp(
             (uint8_t)MOTOR_MODBUS_SLAVE_ID,
             MOTOR_MODBUS_REG_HEARTBEAT,
             motor_modbus_heartbeat_value,
             timeout_ms);
}
