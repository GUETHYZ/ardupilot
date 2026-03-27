#include "MESSAGE_RT_SEND.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

namespace {

// ============================
// 可调参数
// ============================

static constexpr uint8_t GPS_DISABLE_RC_CHANNEL_INDEX = 5;   // CH6
static constexpr uint16_t GPS_DISABLE_PWM_HIGH = 1700;
static constexpr uint16_t GPS_DISABLE_PWM_LOW  = 1300;

static constexpr bool BENCH_FORCE_GS_ENABLE = false;
static constexpr float BENCH_FORCE_GS_MPS   = 5.0f;

static bool gps_disable_switch_active()
{
    if (hal.rcin == nullptr) {
        return false;
    }
    const uint16_t pwm = hal.rcin->read(GPS_DISABLE_RC_CHANNEL_INDEX);
    return pwm > GPS_DISABLE_PWM_HIGH;
}

} // namespace

MESSAGE_RT_SEND::MESSAGE_RT_SEND() :
    uart(nullptr)
{
}

void MESSAGE_RT_SEND::init()
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_MESSAGE_RT, 0);

    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "MSGRT_SEND UART FAIL");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "MSGRT_SEND UART OK");
}

void MESSAGE_RT_SEND::update()
{
    if (uart == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms < send_interval_ms) {
        return;
    }
    last_send_ms = now_ms;

    AP_AHRS &ahrs = AP::ahrs();
    AP_GPS &gps = AP::gps();
    Location loc;
    ahrs.get_location(loc);

    float velN = 0.0f;
    float velE = 0.0f;
    float groundspeed = 0.0f;
    uint8_t flags = 0;

    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned)) {
        velN = vel_ned.x;
        velE = vel_ned.y;
        groundspeed = safe_sqrt(velN * velN + velE * velE);
        flags |= FLAG_GS_VALID | FLAG_VEL_VALID;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_2D) {
        groundspeed = gps.ground_speed();
        const Vector3f &gps_vel = gps.velocity();
        velN = gps_vel.x;
        velE = gps_vel.y;
        flags |= FLAG_GS_VALID | FLAG_VEL_VALID;
    }

    if (BENCH_FORCE_GS_ENABLE) {
        groundspeed = BENCH_FORCE_GS_MPS;
        velN = 0.0f;
        velE = 0.0f;
        flags |= FLAG_GS_VALID;
        flags &= ~FLAG_VEL_VALID;
    }

    float yaw_deg = wrap_360(degrees(ahrs.get_yaw()));
    if (isfinite(yaw_deg)) {
        flags |= FLAG_YAW_VALID;
    } else {
        yaw_deg = 0.0f;
    }

    const bool gps_disable_req = gps_disable_switch_active();
    if (gps_disable_req) {
        flags |= FLAG_GPS_DISABLE_REQ;
    }

    const bool ok = send_packet(now_ms, groundspeed, yaw_deg, loc.lat, loc.lng, flags);

    static uint32_t last_print_ms = 0;
    if (now_ms - last_print_ms >= 8000) {
        gcs().send_text(
            ok ? MAV_SEVERITY_INFO : MAV_SEVERITY_WARNING,
            "MSGRT_SEND gs=%.2f yaw=%.1f vN=%.2f vE=%.2f lat=%.7f lon=%.7f flg=0x%02X gps_dis=%u",
            groundspeed,
            yaw_deg,
            velN,
            velE,
            loc.lat * 1e-7f,
            loc.lng * 1e-7f,
            (unsigned)flags,
            gps_disable_req ? 1U : 0U);
        last_print_ms = now_ms;
    }
}

bool MESSAGE_RT_SEND::send_packet(uint32_t boot_ms,
                                  float groundspeed_mps,
                                  float yaw_deg,
                                  int32_t main_lat_e7,
                                  int32_t main_lon_e7,
                                  uint8_t flags)
{
    uint8_t buf[24];
    buf[0] = HEAD0;
    buf[1] = HEAD1;

    u32_to_be_bytes(boot_ms,         &buf[2]);
    float_to_be_bytes(groundspeed_mps, &buf[6]);
    float_to_be_bytes(yaw_deg,         &buf[10]);
    i32_to_be_bytes(main_lat_e7,       &buf[14]);
    i32_to_be_bytes(main_lon_e7,       &buf[18]);
    buf[22] = flags;

    uint8_t sum = 0;
    for (uint8_t i = 2; i < 23; i++) {
        sum += buf[i];
    }
    buf[23] = uint8_t(~sum);

    const size_t written = uart->write(buf, sizeof(buf));
    return written == sizeof(buf);
}

void MESSAGE_RT_SEND::u32_to_be_bytes(uint32_t value, uint8_t bytes[4])
{
    bytes[0] = (value >> 24) & 0xFF;
    bytes[1] = (value >> 16) & 0xFF;
    bytes[2] = (value >> 8) & 0xFF;
    bytes[3] = value & 0xFF;
}

void MESSAGE_RT_SEND::i32_to_be_bytes(int32_t value, uint8_t bytes[4])
{
    const uint32_t u = (uint32_t)value;
    bytes[0] = (u >> 24) & 0xFF;
    bytes[1] = (u >> 16) & 0xFF;
    bytes[2] = (u >> 8) & 0xFF;
    bytes[3] = u & 0xFF;
}

void MESSAGE_RT_SEND::float_to_be_bytes(float value, uint8_t bytes[4])
{
    union {
        float f;
        uint32_t i;
    } u;

    u.f = value;
    bytes[0] = (u.i >> 24) & 0xFF;
    bytes[1] = (u.i >> 16) & 0xFF;
    bytes[2] = (u.i >> 8) & 0xFF;
    bytes[3] = u.i & 0xFF;
}