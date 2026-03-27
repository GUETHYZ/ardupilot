#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include "MESSAGE_RT_RECEIVE.h"

class GPS_SEND
{
public:
    GPS_SEND(MESSAGE_RT_RECEIVE* msg_rt = nullptr);

    int32_t main_loc_lat_e7 = 0;   // 主飞控纬度，deg * 1e7
    int32_t main_loc_lon_e7 = 0;   // 主飞控经度，deg * 1e7

    void init();
    void update();

private:
    AP_HAL::UARTDriver *uart = nullptr;

    uint32_t last_send_ms = 0;
    uint32_t send_interval_ms = 50;   // 20Hz，先和主飞控对齐

    void float_to_be_bytes(float value, uint8_t bytes[4]) const;
    void log_comparison(uint64_t time_us);

    MESSAGE_RT_RECEIVE* msg_rt_receive = nullptr;
};