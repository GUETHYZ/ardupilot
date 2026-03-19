#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

class MESSAGE_RT_SEND
{
public:
    MESSAGE_RT_SEND();

    void init();
    void update();

private:
    static constexpr uint8_t HEAD0 = 0x5A;
    static constexpr uint8_t HEAD1 = 0xA5;

    // 数据有效位
    static constexpr uint8_t FLAG_GS_VALID         = 1U << 0;
    static constexpr uint8_t FLAG_YAW_VALID        = 1U << 1;
    static constexpr uint8_t FLAG_VEL_VALID        = 1U << 2;

    // 控制请求位
    static constexpr uint8_t FLAG_GPS_DISABLE_REQ  = 1U << 3;

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_send_ms = 0;
    uint32_t send_interval_ms = 50;   // 20Hz

    static void u32_to_be_bytes(uint32_t value, uint8_t bytes[4]);
    static void float_to_be_bytes(float value, uint8_t bytes[4]);

    bool send_packet(uint32_t boot_ms,
                     float groundspeed_mps,
                     float yaw_deg,
                     float velN_mps,
                     float velE_mps,
                     uint8_t flags);
};
