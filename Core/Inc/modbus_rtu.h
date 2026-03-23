#ifndef __MODBUS_RTU_H__
#define __MODBUS_RTU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/**
 * @brief ModBus 目标 UART 端口定义
 */
#define MODBUS_UART_PORT_USART1  (1U)
#define MODBUS_UART_PORT_USART3  (3U)

/**
 * @brief 通过修改该宏切换 ModBus 对接 UART
 *        可选: MODBUS_UART_PORT_USART1 / MODBUS_UART_PORT_USART3
 */
#ifndef MODBUS_UART_PORT
#define MODBUS_UART_PORT         MODBUS_UART_PORT_USART3
#endif

#define MODBUS_MAX_PDU_SIZE      (252U)
#define MODBUS_MAX_ADU_SIZE      (256U)

#define MODBUS_FUNC_READ_HOLDING_REGISTERS  (0x03U)
#define MODBUS_FUNC_READ_INPUT_REGISTERS    (0x04U)
#define MODBUS_FUNC_WRITE_SINGLE_REGISTER    (0x06U)
#define MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS (0x10U)

typedef enum
{
  MODBUS_OK = 0,
  MODBUS_ERR_PARAM,
  MODBUS_ERR_CRC,
  MODBUS_ERR_TIMEOUT,
  MODBUS_ERR_PROTOCOL,
  MODBUS_ERR_HAL
} modbus_status_t;

void modbus_rtu_init(void);
uint16_t modbus_rtu_crc16(const uint8_t *data, uint16_t len);

modbus_status_t modbus_rtu_send_frame(uint8_t slave_addr,
                                      uint8_t function_code,
                                      const uint8_t *payload,
                                      uint16_t payload_len,
                                      uint32_t timeout_ms);

modbus_status_t modbus_rtu_request(uint8_t slave_addr,
                                   uint8_t function_code,
                                   const uint8_t *tx_payload,
                                   uint16_t tx_payload_len,
                                   uint8_t *rx_adu,
                                   uint16_t rx_adu_size,
                                   uint16_t *rx_adu_len,
                                   uint32_t timeout_ms);

modbus_status_t modbus_rtu_read_holding_registers(uint8_t slave_addr,
                                                  uint16_t start_addr,
                                                  uint16_t quantity,
                                                  uint16_t *register_buf,
                                                  uint16_t register_buf_size,
                                                  uint32_t timeout_ms);

modbus_status_t modbus_rtu_read_input_registers(uint8_t slave_addr,
                                               uint16_t start_addr,
                                               uint16_t quantity,
                                               uint16_t *register_buf,
                                               uint16_t register_buf_size,
                                               uint32_t timeout_ms);

modbus_status_t modbus_rtu_write_single_register(uint8_t slave_addr,
                                                 uint16_t register_addr,
                                                 uint16_t value,
                                                 uint32_t timeout_ms);

modbus_status_t modbus_rtu_write_single_register_no_resp(uint8_t slave_addr,
                                                        uint16_t register_addr,
                                                        uint16_t value,
                                                        uint32_t timeout_ms);

modbus_status_t modbus_rtu_write_multiple_registers(uint8_t slave_addr,
                                                    uint16_t start_addr,
                                                    const uint16_t *register_values,
                                                    uint16_t quantity,
                                                    uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_RTU_H__ */
