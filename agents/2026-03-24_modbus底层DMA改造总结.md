# modbus底层DMA改造总结

## 日期
2026-03-24

## 改动目标
将 ModBus 底层串口通信从轮询模式改为 DMA 模式，保持上层接口不变（motor_modbus 无需改动）。

## 改动文件
- Core/Src/modbus_rtu.c

## 主要改动

### 1. 新增DMA状态变量
在 modbus_rtu.c 增加以下静态状态：
- modbus_uart_tx_done
- modbus_uart_rx_done
- modbus_uart_error

用于标识 DMA 发送完成、接收完成和错误状态。

### 2. 新增等待函数
新增 `modbus_wait_dma_flag`，按超时时间轮询等待 DMA 完成标志：
- 成功返回 `MODBUS_OK`
- 超时返回 `MODBUS_ERR_TIMEOUT`
- UART错误返回 `MODBUS_ERR_HAL`

### 3. 初始化逻辑调整
`modbus_rtu_init` 中新增状态清零：
- tx_done/rx_done/error 置 0

### 4. 发送路径改为DMA
`modbus_rtu_send_frame` 由：
- `HAL_UART_Transmit`（轮询）

改为：
- `HAL_UART_Transmit_DMA`
- 等待 `modbus_uart_tx_done`
- 超时或异常时执行 `HAL_UART_AbortTransmit`
- 额外等待 `UART_FLAG_TC`，确保最后停止位发出

### 5. 接收路径改为DMA
`modbus_rtu_request` 中接收由：
- `HAL_UART_Receive`（轮询）

改为：
- `HAL_UART_Receive_DMA`
- 等待 `modbus_uart_rx_done`
- 超时或异常时执行 `HAL_UART_AbortReceive`

原有地址/功能码/CRC校验逻辑保持不变。

### 6. 新增UART回调
在 modbus_rtu.c 增加以下回调并仅处理 `MODBUS_UART_HANDLE`：
- `HAL_UART_TxCpltCallback`
- `HAL_UART_RxCpltCallback`
- `HAL_UART_ErrorCallback`

## 保持不变内容
- ModBus 组帧逻辑
- CRC16 计算逻辑
- 上层 API 及业务层调用方式

## 验证结果
- 已执行编译错误检查：`Core/Src/modbus_rtu.c` 无错误。

## 注意事项
- 工程中应只保留一套同名 UART 回调实现，避免重复定义。
- 当前 DMA 模式为 Normal，符合 ModBus RTU 单帧请求-应答模型。
