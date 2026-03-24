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

static volatile uint8_t modbus_uart_tx_done = 0U;
static volatile uint8_t modbus_uart_rx_done = 0U;
static volatile uint8_t modbus_uart_error = 0U;

/**
 * @brief 等待DMA完成标志位。
 * @param done_flag 需要等待的完成标志位地址。
 * @param timeout_ms 超时时间，单位毫秒。
 * @retval MODBUS_OK 等待成功。
 * @retval MODBUS_ERR_TIMEOUT 超时。
 * @retval MODBUS_ERR_HAL UART异常。
 */
static modbus_status_t modbus_wait_dma_flag(volatile uint8_t *done_flag,
                                            uint32_t timeout_ms)
{
  uint32_t tick_start = HAL_GetTick();

  while (*done_flag == 0U)
  {
    if (modbus_uart_error != 0U)
    {
      return MODBUS_ERR_HAL;
    }

    if ((HAL_GetTick() - tick_start) >= timeout_ms)
    {
      return MODBUS_ERR_TIMEOUT;
    }
  }

  return MODBUS_OK;
}

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
   * 初始化DMA通信状态标志。
   */
  modbus_uart_tx_done = 0U;
  modbus_uart_rx_done = 0U;
  modbus_uart_error = 0U;
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
  modbus_status_t status;
  uint32_t tick_start = 0U;

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

  modbus_uart_tx_done = 0U;
  modbus_uart_error = 0U;
  hal_status = HAL_UART_Transmit_DMA(MODBUS_UART_HANDLE, adu, frame_len);
  if (hal_status != HAL_OK)
  {
    return modbus_convert_hal_status(hal_status);
  }

  status = modbus_wait_dma_flag(&modbus_uart_tx_done, timeout_ms);
  if (status != MODBUS_OK)
  {
    (void)HAL_UART_AbortTransmit(MODBUS_UART_HANDLE);
    return status;
  }

  /**
   * 等待TC置位，确保最后一个停止位已发出。
   */
  tick_start = HAL_GetTick();
  while (__HAL_UART_GET_FLAG(MODBUS_UART_HANDLE, UART_FLAG_TC) == RESET)
  {
    if ((HAL_GetTick() - tick_start) >= timeout_ms)
    {
      return MODBUS_ERR_TIMEOUT;
    }
  }

  return MODBUS_OK;
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

  modbus_uart_rx_done = 0U;
  modbus_uart_error = 0U;
  hal_status = HAL_UART_Receive_DMA(MODBUS_UART_HANDLE, rx_adu, rx_adu_size);
  if (hal_status != HAL_OK)
  {
    return modbus_convert_hal_status(hal_status);
  }

  status = modbus_wait_dma_flag(&modbus_uart_rx_done, timeout_ms);
  if (status != MODBUS_OK)
  {
    (void)HAL_UART_AbortReceive(MODBUS_UART_HANDLE);
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

/**
 * @brief UART DMA发送完成回调。
 * @param huart UART句柄。
 * @retval 无。
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == MODBUS_UART_HANDLE)
  {
    modbus_uart_tx_done = 1U;
  }
}

/**
 * @brief UART DMA接收完成回调。
 * @param huart UART句柄。
 * @retval 无。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == MODBUS_UART_HANDLE)
  {
    modbus_uart_rx_done = 1U;
  }
}

/**
 * @brief UART错误回调。
 * @param huart UART句柄。
 * @retval 无。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == MODBUS_UART_HANDLE)
  {
    modbus_uart_error = 1U;
  }
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

modbus_status_t modbus_rtu_read_input_registers(uint8_t slave_addr,
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
                              MODBUS_FUNC_READ_INPUT_REGISTERS,
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
