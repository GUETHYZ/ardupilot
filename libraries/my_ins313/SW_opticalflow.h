#pragma once

#include <stdint.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_MSP/msp.h>

#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS.h>
#include <Filter/Filter.h>                     // Filter library
#include <Filter/ModeFilter.h>
#include <AP_Logger/AP_Logger.h>


#define MOSS_VIO_MCU 1


class SW_opticalflow_driver
{
public:
    SW_opticalflow_driver();

    // get singleton instance
    static SW_opticalflow_driver *get_singleton() {
        return _singleton;
    }
    // 接收载体飞控 MAVLink 消息后的三个处理函数
    void handle_vbn_mcu_gps_raw_message(const mavlink_message_t &msg);
    void handle_vbn_mcu_attitude_message(const mavlink_message_t &msg);
    void handle_vbn_mcu_pos_message(const mavlink_message_t &msg);

    // 对外可读接口
    bool ext_mavlink_pos_is_healthy() const { return ext_mavlink_pos_healthy; }
    bool ext_mavlink_att_is_healthy() const { return ext_mavlink_att_healthy; }

    uint32_t get_ext_mavlink_pos_time_ms() const { return ext_mavlink_pos_time_ms; }
    uint32_t get_ext_mavlink_att_time_ms() const { return ext_mavlink_att_time_ms; }

    const Location& get_ext_mavlink_pos() const { return mavlink_ext_pos; }
    const Vector3f& get_ext_mavlink_attitude() const { return ext_mavlink_attitude; }

    uint8_t get_mavlink_ext_fix_type() const { return mavlink_ext_fix_type; }
    int32_t get_mavlink_relative_alt() const { return mavlink_relative_alt; }
    float get_vbn_vio_home_alt() const { return _vbn_vio_home_alt; }


private:

    static SW_opticalflow_driver *_singleton;
    // 外部 GPS / 位置状态
    bool ext_mavlink_pos_healthy = false;
    uint32_t ext_mavlink_pos_time_ms = 0;
    uint8_t mavlink_ext_fix_type = 0;

    // 外部姿态状态
    bool ext_mavlink_att_healthy = false;
    uint32_t ext_mavlink_att_time_ms = 0;

    // 位置、姿态缓存
    Location mavlink_ext_pos {};
    Vector3f ext_mavlink_attitude {};

    // 其他辅助量
    int32_t mavlink_relative_alt = 0;   // mm，来自 GLOBAL_POSITION_INT.relative_alt
    float _vbn_vio_home_alt = 0.0f;     // m，来自 GPS_RAW_INT.alt * 0.1f




};


namespace AP {
    SW_opticalflow_driver *opticalflow_driver();
}
