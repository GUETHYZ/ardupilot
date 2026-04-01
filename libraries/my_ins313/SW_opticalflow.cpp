#include "SW_opticalflow.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Relay/AP_Relay.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Logger/LogStructure.h>

extern const AP_HAL::HAL& hal;

// singleton instance pointer
SW_opticalflow_driver *SW_opticalflow_driver::_singleton = nullptr;

// 真正创建一个全局静态实例
static SW_opticalflow_driver g_sw_opticalflow_driver;

SW_opticalflow_driver::SW_opticalflow_driver()
{
    _singleton = this;
}

void SW_opticalflow_driver::handle_vbn_mcu_gps_raw_message(const mavlink_message_t &msg)
{
    mavlink_gps_raw_int_t packet {};
    mavlink_msg_gps_raw_int_decode(&msg, &packet);

    if (!check_latlng(packet.lat, packet.lon)) {
        return;
    }

    mavlink_ext_fix_type = packet.fix_type;
    if (mavlink_ext_fix_type < 3) {
        return;
    }

    AP::gps().set_gps_use_ext_gnss_status(true);
    ext_mavlink_pos_healthy = true;
    ext_mavlink_pos_time_ms = AP_HAL::millis();

    mavlink_ext_pos.alt = packet.alt * 0.1f;
    mavlink_ext_pos.lat = packet.lat;
    mavlink_ext_pos.lng = packet.lon;
    _vbn_vio_home_alt = packet.alt * 0.1f;

    static uint32_t last_debug_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_debug_ms > 1000) {
        last_debug_ms = now;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "GPS_RAW fix=%u lat=%ld lon=%ld alt=%.1f",
                        (unsigned)mavlink_ext_fix_type,
                        (long)packet.lat,
                        (long)packet.lon,
                        (double)(packet.alt * 0.1f));
    }
}

void SW_opticalflow_driver::handle_vbn_mcu_attitude_message(const mavlink_message_t &msg)
{
    mavlink_attitude_t packet {};
    mavlink_msg_attitude_decode(&msg, &packet);

    ext_mavlink_att_healthy = true;
    ext_mavlink_att_time_ms = AP_HAL::millis();

    ext_mavlink_attitude.x = packet.pitch;
    ext_mavlink_attitude.y = packet.roll;
    ext_mavlink_attitude.z = packet.yaw;

    static uint32_t last_debug_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_debug_ms > 1000) {
        last_debug_ms = now;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "ATT pitch=%.3f roll=%.3f yaw=%.3f",
                        (double)packet.pitch,
                        (double)packet.roll,
                        (double)packet.yaw);
    }
}

void SW_opticalflow_driver::handle_vbn_mcu_pos_message(const mavlink_message_t &msg)
{
    mavlink_global_position_int_t packet {};
    mavlink_msg_global_position_int_decode(&msg, &packet);

    if (!check_latlng(packet.lat, packet.lon)) {
        return;
    }

    mavlink_relative_alt = packet.relative_alt;

    static uint32_t last_debug_ms = 0;
    const uint32_t now = AP_HAL::millis();
    if (now - last_debug_ms > 1000) {
        last_debug_ms = now;
        gcs().send_text(MAV_SEVERITY_INFO,
                        "GPOS lat=%ld lon=%ld rel_alt=%ld",
                        (long)packet.lat,
                        (long)packet.lon,
                        (long)packet.relative_alt);
    }
}

namespace AP
{
    SW_opticalflow_driver *opticalflow_driver()
    {
        return SW_opticalflow_driver::get_singleton();
    }
}