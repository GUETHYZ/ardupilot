#include "GPS_SEND.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

GPS_SEND::GPS_SEND() :
    uart(nullptr)
{
}

void GPS_SEND::init(void)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_MY_LINS313, 0);

    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS_SEND: no UART found");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPS_SEND: UART initialized");
}

void GPS_SEND::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();

    static uint32_t last_no_uart_ms = 0;
    static uint32_t last_no_gps_ms = 0;
    static uint32_t last_invalid_gps_ms = 0;
    static uint32_t last_print_ms = 0;

    if (now_ms - last_send_ms < send_interval_ms) {
        return;
    }
    last_send_ms = now_ms;

    if (uart == nullptr)
    {
        if (now_ms - last_no_uart_ms > 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS_SEND: UART not ready");
            last_no_uart_ms = now_ms;
        }
        return;
    }

    AP_GPS &gps = AP::gps();

    if (gps.status() < AP_GPS::GPS_OK_FIX_3D)
    {
        if (now_ms - last_no_gps_ms > 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "GPS_SEND: GPS no 3D fix");
            last_no_gps_ms = now_ms;
        }
        return;
    }

    const Location &loc = gps.location();
    const Vector3f vel = gps.velocity();

    if (loc.lat == 0 || loc.lng == 0)
    {
        if (now_ms - last_invalid_gps_ms > 1000) {
            GCS_SEND_TEXT(
                MAV_SEVERITY_WARNING,
                "GPS_SEND: invalid GPS lat=%.6f lon=%.6f",
                loc.lat * 1e-7f,
                loc.lng * 1e-7f
            );
            last_invalid_gps_ms = now_ms;
        }
        return;
    }

    AP_AHRS &ahrs = AP::ahrs();

    // 这里发送的是主飞控当前 AHRS/EKF 的 yaw，单位：弧度
    const float yaw_rad = ahrs.get_yaw();

    float data[GPS_SEND_FLOAT_NUM];
    data[0] = yaw_rad;              // 原来的 time 位置，现在改成 yaw(rad)
    data[1] = loc.lat * 1e-7f;
    data[2] = loc.lng * 1e-7f;
    data[3] = loc.alt * 0.01f;
    data[4] = vel.x;
    data[5] = vel.y;
    data[6] = vel.z;

    if (now_ms - last_print_ms > 1000)
    {
        GCS_SEND_TEXT(
            MAV_SEVERITY_INFO,
            "GPS_SEND yaw=%.1f lat=%.6f lon=%.6f alt=%.2f",
            degrees(yaw_rad),
            data[1],
            data[2],
            data[3]
        );
        last_print_ms = now_ms;
    }

    uint8_t buf[31];
    buf[0] = 0x7F;
    buf[1] = 0x94;

    float_to_be_bytes(data[0], &buf[2]);   // yaw
    float_to_be_bytes(data[1], &buf[6]);   // lat
    float_to_be_bytes(data[2], &buf[10]);  // lon
    float_to_be_bytes(data[3], &buf[14]);  // alt
    float_to_be_bytes(data[4], &buf[18]);  // velN
    float_to_be_bytes(data[5], &buf[22]);  // velE
    float_to_be_bytes(data[6], &buf[26]);  // velD

    uint8_t sum = 0;
    for (uint8_t i = 2; i < 30; i++) {
        sum += buf[i];
    }
    buf[30] = (uint8_t)(~sum);

    const size_t written = uart->write(buf, sizeof(buf));

    if (written != sizeof(buf))
    {
        GCS_SEND_TEXT(
            MAV_SEVERITY_WARNING,
            "GPS_SEND partial %u/%u",
            (unsigned)written,
            (unsigned)sizeof(buf)
        );
    }
}

void GPS_SEND::float_to_be_bytes(float value, uint8_t bytes[4]) const
{
    union
    {
        float f;
        uint32_t i;
    } u;

    u.f = value;

    bytes[0] = (u.i >> 24) & 0xFF;
    bytes[1] = (u.i >> 16) & 0xFF;
    bytes[2] = (u.i >> 8) & 0xFF;
    bytes[3] = u.i & 0xFF;
}