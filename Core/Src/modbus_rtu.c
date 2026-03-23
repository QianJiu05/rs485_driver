#include "modbus_rtu.h"

#include <string.h>

#if (MODBUS_UART_PORT == MODBUS_UART_PORT_USART1)
extern UART_HandleTypeDef huart1;
#define MODBUS_UART_HANDLE   (&huart1)
#elif (MODBUS_UART_PORT == MODBUS_UART_PORT_USART3)
extern UART_HandleTypeDef huart3;
#define MODBUS_UART_HANDLE   (&huart3)
#else
#error "MODBUS_UART_PORT 配置无效, 仅支持 USART1/USART3"
#endif

static modbus_status_t modbus_convert_hal_status(HAL_StatusTypeDef hal_status)
{
  if (hal_status == HAL_OK)
  {
    return MODBUS_OK;
  }

  if (hal_status == HAL_TIMEOUT)
  {
    return MODBUS_ERR_TIMEOUT;
  }

  return MODBUS_ERR_HAL;
}

void modbus_rtu_init(void)
{
  /**
   * 当前版本无需额外初始化。
   * 如后续接入 485 方向控制或 DMA，可在此扩展。
   */
}

uint16_t modbus_rtu_crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;
  uint16_t i = 0U;
  uint8_t bit = 0U;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < len; i++)
  {
    crc ^= (uint16_t)data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

modbus_status_t modbus_rtu_send_frame(uint8_t slave_addr,
                                      uint8_t function_code,
                                      const uint8_t *payload,
                                      uint16_t payload_len,
                                      uint32_t timeout_ms)
{
  uint8_t adu[MODBUS_MAX_ADU_SIZE] = {0};
  uint16_t frame_len = 0U;
  uint16_t crc = 0U;
  HAL_StatusTypeDef hal_status;

  if ((payload_len > 0U) && (payload == NULL))
  {
    return MODBUS_ERR_PARAM;
  }

  if ((uint16_t)(payload_len + 4U) > MODBUS_MAX_ADU_SIZE)
  {
    return MODBUS_ERR_PARAM;
  }

  adu[0] = slave_addr;
  adu[1] = function_code;

  if (payload_len > 0U)
  {
    (void)memcpy(&adu[2], payload, payload_len);
  }

  frame_len = (uint16_t)(2U + payload_len);
  crc = modbus_rtu_crc16(adu, frame_len);
  adu[frame_len] = (uint8_t)(crc & 0x00FFU);
  adu[frame_len + 1U] = (uint8_t)((crc >> 8U) & 0x00FFU);
  frame_len = (uint16_t)(frame_len + 2U);

  hal_status = HAL_UART_Transmit(MODBUS_UART_HANDLE, adu, frame_len, timeout_ms);
  return modbus_convert_hal_status(hal_status);
}

modbus_status_t modbus_rtu_request(uint8_t slave_addr,
                                   uint8_t function_code,
                                   const uint8_t *tx_payload,
                                   uint16_t tx_payload_len,
                                   uint8_t *rx_adu,
                                   uint16_t rx_adu_size,
                                   uint16_t *rx_adu_len,
                                   uint32_t timeout_ms)
{
  modbus_status_t status;
  uint16_t crc_rx = 0U;
  uint16_t crc_calc = 0U;
  HAL_StatusTypeDef hal_status;

  if ((rx_adu == NULL) || (rx_adu_len == NULL) || (rx_adu_size < 5U))
  {
    return MODBUS_ERR_PARAM;
  }

  status = modbus_rtu_send_frame(slave_addr, function_code, tx_payload, tx_payload_len, timeout_ms);
  if (status != MODBUS_OK)
  {
    return status;
  }

  /**
   * 采用阻塞接收: 上层传入 rx_adu_size 控制期望长度。
   * 对于不定长报文，建议上层按功能码分两步读取。
   */
  hal_status = HAL_UART_Receive(MODBUS_UART_HANDLE, rx_adu, rx_adu_size, timeout_ms);
  status = modbus_convert_hal_status(hal_status);
  if (status != MODBUS_OK)
  {
    return status;
  }

  *rx_adu_len = rx_adu_size;

  if ((rx_adu[0] != slave_addr) || ((rx_adu[1] & 0x7FU) != function_code))
  {
    return MODBUS_ERR_PROTOCOL;
  }

  crc_rx = (uint16_t)(((uint16_t)rx_adu[rx_adu_size - 1U] << 8U) | rx_adu[rx_adu_size - 2U]);
  crc_calc = modbus_rtu_crc16(rx_adu, (uint16_t)(rx_adu_size - 2U));
  if (crc_rx != crc_calc)
  {
    return MODBUS_ERR_CRC;
  }

  return MODBUS_OK;
}

modbus_status_t modbus_rtu_read_holding_registers(uint8_t slave_addr,
                                                  uint16_t start_addr,
                                                  uint16_t quantity,
                                                  uint16_t *register_buf,
                                                  uint16_t register_buf_size,
                                                  uint32_t timeout_ms)
{
  uint8_t tx_payload[4] = {0};
  uint8_t rx_adu[MODBUS_MAX_ADU_SIZE] = {0};
  uint16_t expect_len = 0U;
  uint16_t rx_len = 0U;
  uint16_t i = 0U;
  modbus_status_t status;

  if ((quantity == 0U) || (quantity > 125U) ||
      (register_buf == NULL) || (register_buf_size < quantity))
  {
    return MODBUS_ERR_PARAM;
  }

  tx_payload[0] = (uint8_t)((start_addr >> 8U) & 0x00FFU);
  tx_payload[1] = (uint8_t)(start_addr & 0x00FFU);
  tx_payload[2] = (uint8_t)((quantity >> 8U) & 0x00FFU);
  tx_payload[3] = (uint8_t)(quantity & 0x00FFU);

  expect_len = (uint16_t)(5U + (quantity * 2U));
  status = modbus_rtu_request(slave_addr,
                              MODBUS_FUNC_READ_HOLDING_REGISTERS,
                              tx_payload,
                              4U,
                              rx_adu,
                              expect_len,
                              &rx_len,
                              timeout_ms);
  if (status != MODBUS_OK)
  {
    return status;
  }

  if ((rx_len != expect_len) || (rx_adu[2] != (uint8_t)(quantity * 2U)))
  {
    return MODBUS_ERR_PROTOCOL;
  }

  for (i = 0U; i < quantity; i++)
  {
    register_buf[i] = (uint16_t)(((uint16_t)rx_adu[3U + (2U * i)] << 8U) |
                                  rx_adu[4U + (2U * i)]);
  }

  return MODBUS_OK;
}

modbus_status_t modbus_rtu_write_single_register(uint8_t slave_addr,
                                                 uint16_t register_addr,
                                                 uint16_t value,
                                                 uint32_t timeout_ms)
{
  uint8_t tx_payload[4] = {0};
  uint8_t rx_adu[8] = {0};
  uint16_t rx_len = 0U;
  modbus_status_t status;

  tx_payload[0] = (uint8_t)((register_addr >> 8U) & 0x00FFU);
  tx_payload[1] = (uint8_t)(register_addr & 0x00FFU);
  tx_payload[2] = (uint8_t)((value >> 8U) & 0x00FFU);
  tx_payload[3] = (uint8_t)(value & 0x00FFU);

  status = modbus_rtu_request(slave_addr,
                              MODBUS_FUNC_WRITE_SINGLE_REGISTER,
                              tx_payload,
                              4U,
                              rx_adu,
                              8U,
                              &rx_len,
                              timeout_ms);
  if (status != MODBUS_OK)
  {
    return status;
  }

  if ((rx_len != 8U) || (memcmp(&rx_adu[2], tx_payload, 4U) != 0))
  {
    return MODBUS_ERR_PROTOCOL;
  }

  return MODBUS_OK;
}

modbus_status_t modbus_rtu_write_single_register_no_resp(uint8_t slave_addr,
                                                        uint16_t register_addr,
                                                        uint16_t value,
                                                        uint32_t timeout_ms)
{
  uint8_t payload[4] = {0};

  payload[0] = (uint8_t)((register_addr >> 8U) & 0x00FFU);
  payload[1] = (uint8_t)(register_addr & 0x00FFU);
  payload[2] = (uint8_t)((value >> 8U) & 0x00FFU);
  payload[3] = (uint8_t)(value & 0x00FFU);

  return modbus_rtu_send_frame(slave_addr,
                               MODBUS_FUNC_WRITE_SINGLE_REGISTER,
                               payload, 4U,
                               timeout_ms);
}

modbus_status_t modbus_rtu_write_multiple_registers(uint8_t slave_addr,
                                                    uint16_t start_addr,
                                                    const uint16_t *register_values,
                                                    uint16_t quantity,
                                                    uint32_t timeout_ms)
{
  uint8_t tx_payload[MODBUS_MAX_PDU_SIZE - 1U] = {0};
  uint8_t rx_adu[8] = {0};
  uint16_t rx_len = 0U;
  uint16_t i = 0U;
  modbus_status_t status;

  if ((register_values == NULL) || (quantity == 0U) || (quantity > 123U))
  {
    return MODBUS_ERR_PARAM;
  }

  if ((uint16_t)(5U + (2U * quantity)) > (MODBUS_MAX_PDU_SIZE - 1U))
  {
    return MODBUS_ERR_PARAM;
  }

  tx_payload[0] = (uint8_t)((start_addr >> 8U) & 0x00FFU);
  tx_payload[1] = (uint8_t)(start_addr & 0x00FFU);
  tx_payload[2] = (uint8_t)((quantity >> 8U) & 0x00FFU);
  tx_payload[3] = (uint8_t)(quantity & 0x00FFU);
  tx_payload[4] = (uint8_t)(2U * quantity);

  for (i = 0U; i < quantity; i++)
  {
    tx_payload[5U + (2U * i)] = (uint8_t)((register_values[i] >> 8U) & 0x00FFU);
    tx_payload[6U + (2U * i)] = (uint8_t)(register_values[i] & 0x00FFU);
  }

  status = modbus_rtu_request(slave_addr,
                              MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS,
                              tx_payload,
                              (uint16_t)(5U + (2U * quantity)),
                              rx_adu,
                              8U,
                              &rx_len,
                              timeout_ms);
  if (status != MODBUS_OK)
  {
    return status;
  }

  if ((rx_len != 8U) ||
      (rx_adu[2] != tx_payload[0]) ||
      (rx_adu[3] != tx_payload[1]) ||
      (rx_adu[4] != tx_payload[2]) ||
      (rx_adu[5] != tx_payload[3]))
  {
    return MODBUS_ERR_PROTOCOL;
  }

  return MODBUS_OK;
}
