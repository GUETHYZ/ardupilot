#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_VisualOdom/AP_VisualOdom.h>

#define LINS313_HEAD0 0x7F
#define LINS313_HEAD1 0x94

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

    // 新协议：
    // data[0] = yaw(rad)
    // data[1] = lat
    // data[2] = lon
    // data[3] = alt
    // data[4] = velN
    // data[5] = velE
    // data[6] = velD
    float yaw = 0.0f;
    float lat = 0.0f;
    float lon = 0.0f;
    float alt = 0.0f;

    float velN = 0.0f;
    float velE = 0.0f;
    float velD = 0.0f;

    bool origin_set = false;
    Location origin;

    // 用前几个有效样本做 origin 平均
    uint8_t origin_count = 0;
    float origin_lat_sum = 0.0f;
    float origin_lon_sum = 0.0f;
    float origin_alt_sum = 0.0f;

    float be_bytes_to_float(const uint8_t bytes[4]) const;
};