#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

// 需要先在 AP_SerialManager.h 中定义协议号，例如：
// #define SERIAL_PROTOCOL_LINS313_SEND 30
// 若未定义，可使用其他未占用的协议号或通过参数直接指定串口

class MY_LINS313_SEND {
public:
    MY_LINS313_SEND();

    /* 初始化串口 */
    void init();

   
    void update();

private:
    AP_HAL::UARTDriver *uart;          // 串口指针
    uint32_t last_send_ms;              // 上次发送时间（毫秒）
    uint32_t send_interval_ms = 100;    // 发送周期（默认100ms = 10Hz）

    /* 将float转换为大端字节序并写入缓冲区 */
    void float_to_be_bytes(float value, uint8_t bytes[4]) const;
};