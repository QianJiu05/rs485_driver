# ModBus驱动实现总结

## 1. 需求目标
- 新增一个可复用的 ModBus RTU 驱动。
- 通过宏定义控制当前对接 UART。
- 保持对现有工程最小侵入。

## 2. 实现内容

### 2.1 新增驱动头文件
文件: `Core/Inc/modbus_rtu.h`
- 提供 UART 选择宏:
  - `MODBUS_UART_PORT_USART1`
  - `MODBUS_UART_PORT_USART3`
  - `MODBUS_UART_PORT`（默认选择 USART1）
- 提供状态码 `modbus_status_t`。
- 提供接口:
  - `modbus_rtu_init`
  - `modbus_rtu_crc16`
  - `modbus_rtu_send_frame`
  - `modbus_rtu_request`
  - `modbus_rtu_read_holding_registers`
  - `modbus_rtu_write_single_register`

### 2.2 新增驱动源文件
文件: `Core/Src/modbus_rtu.c`
- 根据 `MODBUS_UART_PORT` 自动绑定到 `huart1` 或 `huart3`。
- 实现 ModBus RTU CRC16 校验。
- 实现基础报文发送与阻塞式请求/响应。
- 实现常用功能码:
  - 0x03 读保持寄存器
  - 0x06 写单个寄存器
- 增加参数检查、CRC校验和协议字段校验。

### 2.3 工程接入
文件: `Core/Src/main.c`
- 在用户区引入 `modbus_rtu.h`。
- 在初始化阶段调用 `modbus_rtu_init()`。
- 在主循环用户区加入注释掉的调用示例，避免默认阻塞。

## 3. UART切换方式
在 `Core/Inc/modbus_rtu.h` 中修改如下宏即可:

```c
#define MODBUS_UART_PORT MODBUS_UART_PORT_USART1
```

可切换为:

```c
#define MODBUS_UART_PORT MODBUS_UART_PORT_USART3
```

## 4. 说明
- 本次实现采用阻塞式 HAL UART 接口，便于快速落地。
- 若后续需要更高实时性，可扩展为中断或 DMA 模式。
- 当前未修改 CubeMX 自动生成的串口初始化逻辑。
