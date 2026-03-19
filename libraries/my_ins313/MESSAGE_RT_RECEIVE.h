#pragma once

#include <AP_HAL/AP_HAL.h>

class MESSAGE_RT_RECEIVE
{
public:
    MESSAGE_RT_RECEIVE();

    void init();
    void update();

    bool healthy() const;
    bool speed_valid() const;
    bool yaw_valid() const;
    bool vel_valid() const;

    float get_groundspeed_mps() const { return groundspeed_mps; }
    float get_yaw_deg() const { return yaw_deg; }
    float get_velN_mps() const { return velN_mps; }
    float get_velE_mps() const { return velE_mps; }
    uint32_t get_last_rx_ms() const { return last_rx_ms; }
    bool get_gps_disable_req() const { return gps_disable_req; }
    bool get_gps_disable_applied() const { return gps_disable_applied; }

private:
    static constexpr uint8_t HEAD0 = 0x5A;
    static constexpr uint8_t HEAD1 = 0xA5;

    static constexpr uint8_t FLAG_GS_VALID         = 1U << 0;
    static constexpr uint8_t FLAG_YAW_VALID        = 1U << 1;
    static constexpr uint8_t FLAG_VEL_VALID        = 1U << 2;
    static constexpr uint8_t FLAG_GPS_DISABLE_REQ  = 1U << 3;

    AP_HAL::UARTDriver *uart = nullptr;

    uint16_t recv_count = 0;
    uint8_t sum = 0;
    uint8_t payload[21]{};

    uint32_t peer_boot_ms = 0;
    uint32_t last_rx_ms = 0;
    uint32_t timeout_ms = 300;

    float groundspeed_mps = 0.0f;
    float yaw_deg = 0.0f;
    float velN_mps = 0.0f;
    float velE_mps = 0.0f;
    uint8_t flags = 0;

    bool gps_disable_req = false;
    bool gps_disable_applied = false;
    uint32_t last_aux_apply_ms = 0;

    static uint32_t be_bytes_to_u32(const uint8_t bytes[4]);
    static float be_bytes_to_float(const uint8_t bytes[4]);

    void handle_packet();
    void inject_virtual_airspeed();
    void apply_control_requests();
    void set_gps_disable(bool enable);
};
