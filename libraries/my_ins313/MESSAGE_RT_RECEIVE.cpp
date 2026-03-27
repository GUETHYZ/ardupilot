#include "MESSAGE_RT_RECEIVE.h"

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_VisualOdom/AP_VisualOdom.h>

extern const AP_HAL::HAL& hal;

MESSAGE_RT_RECEIVE::MESSAGE_RT_RECEIVE()
{
}

void MESSAGE_RT_RECEIVE::init()
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_MESSAGE_RT, 0);

    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "MSGRT_RECV UART FAIL");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "MSGRT_RECV UART OK");
}

void MESSAGE_RT_RECEIVE::update()
{
    if (uart == nullptr) {
        return;
    }

    while (uart->available()) {
        const uint8_t c = uart->read();

        switch (recv_count) {
        case 0:
            recv_count = (c == HEAD0) ? 1 : 0;
            break;

        case 1:
            if (c == HEAD1) {
                recv_count = 2;
                sum = 0;
            } else {
                recv_count = 0;
            }
            break;

        default:
            if (recv_count < 23) {
                payload[recv_count - 2] = c;
                sum += c;
                recv_count++;
            } else {
                const uint8_t checksum = uint8_t(~sum);
                if (checksum == c) {
                    handle_packet();
                }
                recv_count = 0;
            }
            break;
        }
    }

    // 缓存值持续注入 airspeed
    inject_virtual_airspeed();

    //把主飞控传入的yaw注入成ExternalNav yaw
    inject_external_yaw();

    // 处理主飞控发来的控制请求（当前只做 GPS Disable）
    apply_control_requests();
}

bool MESSAGE_RT_RECEIVE::healthy() const
{
    return (AP_HAL::millis() - last_rx_ms) < timeout_ms;
}

bool MESSAGE_RT_RECEIVE::speed_valid() const
{
    return healthy() && ((flags & FLAG_GS_VALID) != 0);
}

bool MESSAGE_RT_RECEIVE::yaw_valid() const
{
    return healthy() && ((flags & FLAG_YAW_VALID) != 0);
}

bool MESSAGE_RT_RECEIVE::vel_valid() const
{
    return healthy() && ((flags & FLAG_VEL_VALID) != 0);
}

uint32_t MESSAGE_RT_RECEIVE::be_bytes_to_u32(const uint8_t bytes[4])
{
    return (uint32_t(bytes[0]) << 24) |
           (uint32_t(bytes[1]) << 16) |
           (uint32_t(bytes[2]) << 8)  |
           uint32_t(bytes[3]);
}

float MESSAGE_RT_RECEIVE::be_bytes_to_float(const uint8_t bytes[4])
{
    union {
        uint32_t i;
        float f;
    } u;

    u.i = be_bytes_to_u32(bytes);
    return u.f;
}

void MESSAGE_RT_RECEIVE::handle_packet()
{
    peer_boot_ms        = be_bytes_to_u32(&payload[0]);
    groundspeed_mps     = be_bytes_to_float(&payload[4]);
    yaw_deg             = be_bytes_to_float(&payload[8]);
    from_main_loc_lat_e7 = be_bytes_to_i32(&payload[12]);
    from_main_loc_lon_e7 = be_bytes_to_i32(&payload[16]);
    flags               = payload[20];
    last_rx_ms          = AP_HAL::millis();
    gps_disable_req     = (flags & FLAG_GPS_DISABLE_REQ) != 0;

    static uint32_t last_print_ms = 0;
    if (last_rx_ms - last_print_ms >= 1000) {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "MSGRT_RECV gs=%.2f yaw=%.1f lat=%.7f lon=%.7f flg=0x%02X h=%u gps_dis=%u",
            groundspeed_mps,
            yaw_deg,
            from_main_loc_lat_e7 * 1e-7f,
            from_main_loc_lon_e7 * 1e-7f,
            (unsigned)flags,
            healthy() ? 1U : 0U,
            gps_disable_req ? 1U : 0U);
        last_print_ms = last_rx_ms;
    }
}
void MESSAGE_RT_RECEIVE::inject_virtual_airspeed()
{
#if AP_AIRSPEED_ENABLED && AP_AIRSPEED_EXTERNAL_ENABLED
    const uint32_t now_ms = AP_HAL::millis();

    AP_Airspeed *airspeed = AP::airspeed();
    if (airspeed == nullptr || !airspeed->enabled()) {
        static uint32_t last_warn_ms = 0;
        if (now_ms - last_warn_ms >= 2000) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "MSGRT_RECV no ARSPD backend (set ARSPD_TYPE=16)");
            last_warn_ms = now_ms;
        }
        return;
    }

    // 100Hz 注入缓存值
    static uint32_t last_inject_ms = 0;
    if (now_ms - last_inject_ms < 10) {
        return;
    }
    last_inject_ms = now_ms;

    const float ratio = MAX(0.1f, airspeed->get_airspeed_ratio());

    // 启动期先注入 0 压差，避免污染原生 offset 校准
    float gs_for_dp = 0.0f;
    const bool msg_ok = healthy() && ((flags & FLAG_GS_VALID) != 0);
    if (now_ms > 12000 && msg_ok) {
        gs_for_dp = MAX(0.0f, groundspeed_mps);
    }

    AP_ExternalAHRS::airspeed_data_message_t pkt{};
    pkt.differential_pressure = (gs_for_dp * gs_for_dp) / ratio;
    pkt.temperature = 25.0f;

    airspeed->handle_external(pkt);

    // static uint32_t last_dbg_ms = 0;
    // if (now_ms - last_dbg_ms >= 3000) {
    //     gcs().send_text(
    //         MAV_SEVERITY_INFO,
    //         "AIRSPD_INJ gs=%.2f ratio=%.2f dp=%.2f | ASPD healthy=%u air=%.2f rawdp=%.2f",
    //         gs_for_dp,
    //         ratio,
    //         pkt.differential_pressure,
    //         airspeed->healthy() ? 1U : 0U,
    //         airspeed->get_airspeed(),
    //         airspeed->get_differential_pressure());
    //     last_dbg_ms = now_ms;
    // }
#else
    static uint32_t last_warn_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_warn_ms >= 2000) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "MSGRT_RECV build has no external airspeed support");
        last_warn_ms = now_ms;
    }
#endif
}

void MESSAGE_RT_RECEIVE::apply_control_requests()
{
    // 链路超时则自动释放 GPS Disable，请求回到“LOW”状态
    const bool desired_disable = healthy() && gps_disable_req;

    if (desired_disable == gps_disable_applied) {
        return;
    }

    // 避免异常抖动下过快重复触发
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_aux_apply_ms < 200) {
        return;
    }
    last_aux_apply_ms = now_ms;

    set_gps_disable(desired_disable);
}

void MESSAGE_RT_RECEIVE::set_gps_disable(bool enable)
{
#if AP_RC_CHANNEL_ENABLED && AP_GPS_ENABLED
    const bool ok = rc().run_aux_function(
        RC_Channel::AUX_FUNC::GPS_DISABLE,
        enable ? RC_Channel::AuxSwitchPos::HIGH : RC_Channel::AuxSwitchPos::LOW,
        RC_Channel::AuxFuncTriggerSource::SCRIPTING);

    static uint32_t last_log_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();

    if (ok) {
        gps_disable_applied = enable;
        if (now_ms - last_log_ms >= 500 || true) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "MSGRT GPS_DISABLE %s",
                            enable ? "ON" : "OFF");
            last_log_ms = now_ms;
        }
    } else {
        if (now_ms - last_log_ms >= 500) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "MSGRT GPS_DISABLE %s FAIL",
                            enable ? "ON" : "OFF");
            last_log_ms = now_ms;
        }
    }
#else
    (void)enable;
    static uint32_t last_warn_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_warn_ms >= 2000) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "MSGRT GPS_DISABLE unsupported in this build");
        last_warn_ms = now_ms;
    }
#endif
}

void MESSAGE_RT_RECEIVE::inject_external_yaw()
{
#if HAL_VISUALODOM_ENABLED
    const uint32_t now_ms = AP_HAL::millis();

    // 最近必须收到过主飞控数据
    if (!healthy()) {
        return;
    }

    // yaw 必须有效
    if ((flags & FLAG_YAW_VALID) == 0) {
        return;
    }

    auto *viso = AP::visualodom();
    if (viso == nullptr || !viso->enabled()) {
        static uint32_t last_warn_ms = 0;
        if (now_ms - last_warn_ms >= 2000) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "EXT_YAW: visualodom not enabled (set VISO_TYPE=1)");
            last_warn_ms = now_ms;
        }
        return;
    }

    // 50Hz 注入一次即可
    static uint32_t last_yaw_inject_ms = 0;
    if (now_ms - last_yaw_inject_ms < 20) {
        return;
    }
    last_yaw_inject_ms = now_ms;

    // 用测试飞控自身 roll/pitch，叠加主飞控发来的 yaw
    AP_AHRS &ahrs = AP::ahrs();
    const float roll = ahrs.get_roll();
    const float pitch = ahrs.get_pitch();
    const float yaw_rad = radians(yaw_deg);

    // 这里不想让 ExternalNav 位置影响当前 GPS 位置源，所以位置给 0，
    // 同时把位置误差给很大，只突出 yaw 的作用
    constexpr float x = 0.0f;
    constexpr float y = 0.0f;
    constexpr float z = 0.0f;
    constexpr float posErr = 100.0f;       // 很大，表示位置不可信
    const float angErr = radians(5.0f);    // yaw 误差 5 度，可再调
    constexpr uint8_t reset_counter = 0;
    constexpr int8_t quality = 100;

    viso->handle_pose_estimate(
        0,              // remote_time_us
        now_ms,         // local time_ms
        x, y, z,
        roll, pitch, yaw_rad,
        posErr,
        angErr,
        reset_counter,
        quality
    );

    static uint32_t last_dbg_ms = 0;
    if (now_ms - last_dbg_ms >= 1000) {
        gcs().send_text(MAV_SEVERITY_INFO,
                        "EXT_YAW yaw=%.1f roll=%.1f pitch=%.1f",
                        yaw_deg,
                        degrees(roll),
                        degrees(pitch));
        last_dbg_ms = now_ms;
    }
#else
    static uint32_t last_warn_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_warn_ms >= 2000) {
        gcs().send_text(MAV_SEVERITY_WARNING,
                        "EXT_YAW: HAL_VISUALODOM not enabled in build");
        last_warn_ms = now_ms;
    }
#endif
}

int32_t MESSAGE_RT_RECEIVE::be_bytes_to_i32(const uint8_t bytes[4])
{
    return int32_t(
        (uint32_t(bytes[0]) << 24) |
        (uint32_t(bytes[1]) << 16) |
        (uint32_t(bytes[2]) << 8)  |
        uint32_t(bytes[3]));
}
