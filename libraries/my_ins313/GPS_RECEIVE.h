#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_VisualOdom/AP_VisualOdom.h>
#include <AP_Logger/AP_Logger.h>

#define GPS_HEAD0 0xAA
#define GPS_HEAD1 0x55

#ifndef GPS_RECEIVE_ENABLED
#define GPS_RECEIVE_ENABLED 1
#endif

#pragma pack(push,1)
struct GPSDataPacket
{
    uint8_t data[28];
};
#pragma pack(pop)

class GPS_RECEIVE
{
public:

    GPS_RECEIVE();

    void init();
    void update();

private:

    AP_HAL::UARTDriver *uart = nullptr;

    GPSDataPacket pack;

    uint16_t recv_count = 0;
    uint8_t sum = 0;

    // float lat = 0;
    // float lon = 0;
    // float alt = 0;

    // float velN = 0;
    // float velE = 0;
    // float velD = 0;
    uint32_t remote_boot_ms = 0;

    float remote_lat_deg = 0.0f;
    float remote_lon_deg = 0.0f;
    float remote_alt_m = 0.0f;

    float remote_vel_n_mps = 0.0f;
    float remote_vel_e_mps = 0.0f;
    float remote_vel_d_mps = 0.0f;



    bool origin_set = false;

    Location origin;

    // origin平均
    uint8_t origin_count = 0;
    float origin_lat_sum = 0;
    float origin_lon_sum = 0;
    float origin_alt_sum = 0;

    float be_bytes_to_float(const uint8_t bytes[4]) const;

    void log_compare(uint64_t time_us);


    uint32_t be_bytes_to_u32(const uint8_t byte[4])const;
};