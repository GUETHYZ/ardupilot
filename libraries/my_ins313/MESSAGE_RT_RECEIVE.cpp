#include "MESSAGE_RT_RECEIVE.h"

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

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
    peer_boot_ms     = be_bytes_to_u32(&payload[0]);
    groundspeed_mps  = be_bytes_to_float(&payload[4]);
    yaw_deg          = be_bytes_to_float(&payload[8]);
    velN_mps         = be_bytes_to_float(&payload[12]);
    velE_mps         = be_bytes_to_float(&payload[16]);
    flags            = payload[20];
    last_rx_ms       = AP_HAL::millis();
    gps_disable_req  = (flags & FLAG_GPS_DISABLE_REQ) != 0;

    static uint32_t last_print_ms = 0;
    if (last_rx_ms - last_print_ms >= 1000) {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "MSGRT_RECV gs=%.2f yaw=%.1f vN=%.2f vE=%.2f flg=0x%02X h=%u gps_dis=%u",
            groundspeed_mps,
            yaw_deg,
            velN_mps,
            velE_mps,
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

    static uint32_t last_dbg_ms = 0;
    if (now_ms - last_dbg_ms >= 1000) {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "AIRSPD_INJ gs=%.2f ratio=%.2f dp=%.2f | ASPD healthy=%u air=%.2f rawdp=%.2f",
            gs_for_dp,
            ratio,
            pkt.differential_pressure,
            airspeed->healthy() ? 1U : 0U,
            airspeed->get_airspeed(),
            airspeed->get_differential_pressure());
        last_dbg_ms = now_ms;
    }
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
