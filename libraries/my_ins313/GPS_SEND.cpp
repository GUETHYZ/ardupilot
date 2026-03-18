#include "GPS_SEND.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>

extern const AP_HAL::HAL& hal;

GPS_SEND::GPS_SEND() :
    uart(nullptr)
{
}

void GPS_SEND::init()
{
    auto &sm = AP::serialmanager();

    uart = sm.find_serial(AP_SerialManager::SerialProtocol_MESSAGE_RT, 0);

    if (uart == nullptr)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "EKF_SEND UART FAIL");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "EKF_SEND UART OK");
}

void GPS_SEND::update()
{
    if (uart == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_send_ms < send_interval_ms) {
        return;
    }

    last_send_ms = now_ms;

    static uint32_t last_no_location_print_ms = 0;
    static uint32_t last_pos_print_ms = 0;
    static uint32_t last_partial_print_ms = 0;
    static bool last_location_valid = false;

    AP_AHRS &ahrs = AP::ahrs();

    Location loc;
    if (!ahrs.get_location(loc))
    {
        if (now_ms - last_no_location_print_ms > 1000)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "EKF NO LOCATION");
            last_no_location_print_ms = now_ms;
        }

        last_location_valid = false;
        return;
    }

    if (!last_location_valid)
    {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "EKF LOCATION READY lat=%.6f lon=%.6f alt=%.2f",
            loc.lat * 1e-7f,
            loc.lng * 1e-7f,
            loc.alt * 0.01f
        );
        last_location_valid = true;
    }

    Vector3f velNED;
    if (!ahrs.get_velocity_NED(velNED))
    {
        velNED.zero();
    }

    float data[7];
    data[0] = now_ms;
    data[1] = loc.lat * 1e-7f;
    data[2] = loc.lng * 1e-7f;
    data[3] = loc.alt * 0.01f;
    data[4] = velNED.x;
    data[5] = velNED.y;
    data[6] = velNED.z;

    if (now_ms - last_pos_print_ms > 1000)
    {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "EKF_POS lat=%.6f lon=%.6f alt=%.2f vN=%.2f vE=%.2f vD=%.2f",
            data[1],
            data[2],
            data[3],
            data[4],
            data[5],
            data[6]
        );

        last_pos_print_ms = now_ms;
    }

    uint8_t buf[31];
    buf[0] = 0xAA;
    buf[1] = 0x55;

    float_to_be_bytes(data[0], &buf[2]);
    float_to_be_bytes(data[1], &buf[6]);
    float_to_be_bytes(data[2], &buf[10]);
    float_to_be_bytes(data[3], &buf[14]);
    float_to_be_bytes(data[4], &buf[18]);
    float_to_be_bytes(data[5], &buf[22]);
    float_to_be_bytes(data[6], &buf[26]);

    uint8_t sum = 0;
    for (uint8_t i = 2; i < 30; i++) {
        sum += buf[i];
    }
    buf[30] = (uint8_t)(~sum);

    const size_t written = uart->write(buf, sizeof(buf));

    if (written != sizeof(buf))
    {
        if (now_ms - last_partial_print_ms > 1000)
        {
            gcs().send_text(
                MAV_SEVERITY_WARNING,
                "EKF_SEND PARTIAL %u/%u",
                (unsigned)written,
                (unsigned)sizeof(buf)
            );
            last_partial_print_ms = now_ms;
        }
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