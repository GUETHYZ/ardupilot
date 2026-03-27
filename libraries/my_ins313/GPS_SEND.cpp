#include "GPS_SEND.h"

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL &hal;

AP_AHRS &ahrs = AP::ahrs();
Location loc;

GPS_SEND::GPS_SEND(MESSAGE_RT_RECEIVE *msg_rt) :
    uart(nullptr),
    msg_rt_receive(msg_rt)
{
}

void GPS_SEND::init()
{
    auto &sm = AP::serialmanager();

    uart = sm.find_serial(AP_SerialManager::SerialProtocol_MESSAGE_RT, 0);

    if (uart == nullptr) {
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
    //static uint32_t last_null_ptr_print_ms = 0;
    static bool last_location_valid = false;

    if (!ahrs.get_location(loc)) {
        if (now_ms - last_no_location_print_ms > 1000) {
            gcs().send_text(MAV_SEVERITY_INFO, "EKF NO LOCATION");
            last_no_location_print_ms = now_ms;
        }
        last_location_valid = false;
        return;
    }

    if (!last_location_valid) {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "EKF LOCATION READY lat=%.6f lon=%.6f alt=%.2f",
            loc.lat * 1e-7f,
            loc.lng * 1e-7f,
            loc.alt * 0.01f);
        last_location_valid = true;
    }

    Vector3f velNED;
    if (!ahrs.get_velocity_NED(velNED)) {
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
    if (msg_rt_receive != nullptr) {
        main_loc_lat_e7 = msg_rt_receive->get_from_main_loc_lat_e7();
        main_loc_lon_e7 = msg_rt_receive->get_from_main_loc_lon_e7();
    } else {
        main_loc_lat_e7 = 0;
        main_loc_lon_e7 = 0;
    }
    if (now_ms - last_pos_print_ms > 1000) 
    {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "EKF_POS lat=%.6f lon=%.6f alt=%.2f vN=%.2f vE=%.2f vD=%.2f",
            data[1], data[2], data[3], data[4], data[5], data[6]);

        gcs().send_text(
            MAV_SEVERITY_INFO,
            "GPS_SEND main_lat=%.7f main_lon=%.7f",
            main_loc_lat_e7 * 1e-7f,
            main_loc_lon_e7 * 1e-7f);

        last_pos_print_ms = now_ms;
    }

    log_comparison(AP_HAL::micros64());

    if (written != sizeof(buf)) {
        if (now_ms - last_partial_print_ms > 1000) {
            gcs().send_text(
                MAV_SEVERITY_WARNING,
                "EKF_SEND PARTIAL %u/%u",
                (unsigned)written,
                (unsigned)sizeof(buf));
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
void GPS_SEND::log_comparison(uint64_t time_us)
{
#if HAL_LOGGING_ENABLED
    static bool ref_inited = false;
    static int32_t ref_lat_e7 = 0;
    static int32_t ref_lon_e7 = 0;

    if (!ref_inited) {
        ref_lat_e7 = loc.lat;
        ref_lon_e7 = loc.lng;
        ref_inited = true;
    }

    const float ref_lat_deg = ref_lat_e7 * 1e-7f;
    const float lat_scale = 111319.5f;
    const float lon_scale = 111319.5f * cosf(radians(ref_lat_deg));

    const float remote_n = (loc.lat - ref_lat_e7) * 1e-7f * lat_scale;
    const float remote_e = (loc.lng - ref_lon_e7) * 1e-7f * lon_scale;

    float main_n = 0.0f;
    float main_e = 0.0f;
    if (main_loc_lat_e7 != 0 || main_loc_lon_e7 != 0) {
        main_n = (main_loc_lat_e7 - ref_lat_e7) * 1e-7f * lat_scale;
        main_e = (main_loc_lon_e7 - ref_lon_e7) * 1e-7f * lon_scale;
    } 

    struct log_GPEK_TEST pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPEK_TEST),
        time_us       : time_us,
        remote_lat_e7 : loc.lat,
        remote_lon_e7  : loc.lng,
        main_lat_e7   : main_loc_lat_e7,
        main_lon_e7   : main_loc_lon_e7,
        remote_n      : remote_n,
        remote_e      : remote_e,
        main_n        : main_n,
        main_e        : main_e,
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}