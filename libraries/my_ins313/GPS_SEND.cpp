#include "GPS_SEND.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>

#include <cmath>
#include <cstring>

extern const AP_HAL::HAL& hal;

namespace {

static constexpr uint8_t UBX_SYNC1 = 0xB5;
static constexpr uint8_t UBX_SYNC2 = 0x62;

static constexpr uint8_t UBX_CLASS_NAV = 0x01;
static constexpr uint8_t UBX_CLASS_ACK = 0x05;
static constexpr uint8_t UBX_CLASS_CFG = 0x06;

static constexpr uint8_t UBX_NAV_POSLLH = 0x02;
static constexpr uint8_t UBX_NAV_STATUS = 0x03;
static constexpr uint8_t UBX_NAV_DOP    = 0x04;
static constexpr uint8_t UBX_NAV_VELNED = 0x12;
static constexpr uint8_t UBX_NAV_PVT    = 0x07;

static constexpr uint8_t UBX_ACK_NAK = 0x00;
static constexpr uint8_t UBX_ACK_ACK = 0x01;

static constexpr uint8_t UBX_CFG_PRT    = 0x00;
static constexpr uint8_t UBX_CFG_MSG    = 0x01;
static constexpr uint8_t UBX_CFG_RATE   = 0x08;
static constexpr uint8_t UBX_CFG_NAV5   = 0x24;
static constexpr uint8_t UBX_CFG_VALSET = 0x8A;
static constexpr uint8_t UBX_CFG_VALGET = 0x8B;
static constexpr uint8_t UBX_CFG_VALDEL = 0x8C;

// ============================
// 用户可调参数
// ============================

// 用哪个 RC 通道做 fake GPS 开关
// 0 基索引：CH1->0, CH7->6, CH8->7 ...
static constexpr uint8_t FAKE_GPS_RC_CHANNEL_INDEX = 6;   // CH7

// 拨杆高电平阈值
static constexpr uint16_t FAKE_GPS_PWM_HIGH = 1700;
static constexpr uint16_t FAKE_GPS_PWM_LOW  = 1300;

// 如果室内一上电主飞控就没有真 GPS，可用这个默认锚点
// 下面默认值就是你当前场地附近，按需改
static constexpr int32_t DEFAULT_FAKE_LAT_E7 = 305385000;   // 30.5385000
static constexpr int32_t DEFAULT_FAKE_LON_E7 = 1040560000;  // 104.0560000
static constexpr int32_t DEFAULT_FAKE_ALT_CM = 10000;       // 520m

// 默认假 GPS 质量参数
static constexpr uint8_t  DEFAULT_FAKE_NUM_SATS = 14;
static constexpr uint32_t DEFAULT_FAKE_HACC_MM  = 800;   // 0.8m
static constexpr uint32_t DEFAULT_FAKE_VACC_MM  = 1500;  // 1.5m
static constexpr uint32_t DEFAULT_FAKE_SACC_MMPS = 120;  // 0.12m/s
static constexpr uint32_t DEFAULT_FAKE_HEADACC_1E5 = 500000; // 5deg

// 是否启用脚本运动模式
// false: 锁点模式（最稳，适合室内先骗过测试飞控）
// true : 按固定航向+固定速度直线运动
static constexpr bool FAKE_SCRIPT_MOVE_ENABLE = false;

// 脚本运动参数（只有上面为 true 才生效）
static constexpr float FAKE_SCRIPT_SPEED_MPS   = 0.8f;
static constexpr float FAKE_SCRIPT_HEADING_DEG = 90.0f;

// ============================

#pragma pack(push, 1)
struct UBXNavPosllh {
    uint32_t iTOW;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
};

struct UBXNavStatus {
    uint32_t iTOW;
    uint8_t gpsFix;
    uint8_t flags;
    uint8_t fixStat;
    uint8_t flags2;
    uint32_t ttff;
    uint32_t msss;
};

struct UBXNavDop {
    uint32_t iTOW;
    uint16_t gDOP;
    uint16_t pDOP;
    uint16_t tDOP;
    uint16_t vDOP;
    uint16_t hDOP;
    uint16_t nDOP;
    uint16_t eDOP;
};

struct UBXNavVelned {
    uint32_t iTOW;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t speed;
    uint32_t gSpeed;
    int32_t heading;
    uint32_t sAcc;
    uint32_t cAcc;
};

struct UBXNavPvt {
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;
    uint32_t tAcc;
    int32_t nano;
    uint8_t fixType;
    uint8_t flags;
    uint8_t flags2;
    uint8_t numSV;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t reserved1[6];
    int32_t headVeh;
    int16_t magDec;
    uint16_t magAcc;
};

struct UBXCfgPrt {
    uint8_t portID;
    uint8_t reserved0;
    uint16_t txReady;
    uint32_t mode;
    uint32_t baudRate;
    uint16_t inProtoMask;
    uint16_t outProtoMask;
    uint16_t flags;
    uint16_t reserved5;
};

struct UBXCfgRate {
    uint16_t measRate;
    uint16_t navRate;
    uint16_t timeRef;
};

struct UBXAck {
    uint8_t clsID;
    uint8_t msgID;
};
#pragma pack(pop)

// 统一导航解
struct NavSolution {
    int32_t lat = 0;          // degE7
    int32_t lon = 0;          // degE7
    int32_t alt_cm = 0;       // cm
    float velN = 0.0f;        // m/s
    float velE = 0.0f;        // m/s
    float velD = 0.0f;        // m/s
    float groundspeed = 0.0f; // m/s
    float course_deg = 0.0f;  // deg
    uint8_t fixType = 0;      // 0/2/3
    uint8_t flags = 0;        // gnssFixOK...
    uint8_t numSV = 0;
    uint32_t hAcc_mm = 1500;
    uint32_t vAcc_mm = 2500;
    uint32_t sAcc_mmps = 200;
    uint32_t iTOW = 0;
};

struct FakeState {
    bool active = false;
    bool initialized = false;

    int32_t anchor_lat = DEFAULT_FAKE_LAT_E7;
    int32_t anchor_lon = DEFAULT_FAKE_LON_E7;
    int32_t anchor_alt_cm = DEFAULT_FAKE_ALT_CM;

    float north_m = 0.0f;
    float east_m  = 0.0f;
    float down_m  = 0.0f;

    float heading_deg = FAKE_SCRIPT_HEADING_DEG;
    float speed_mps   = 0.0f;

    uint32_t last_update_ms = 0;
    uint32_t fake_itow_ms   = 0;
};

static NavSolution g_nav_sol{};
static bool g_nav_sol_valid = false;
static FakeState g_fake_state{};

static uint8_t map_fix_type(const AP_GPS::GPS_Status st)
{
    switch (st) {
    case AP_GPS::GPS_OK_FIX_2D:
        return 2;
    case AP_GPS::GPS_OK_FIX_3D:
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        return 3;
    case AP_GPS::NO_FIX:
    case AP_GPS::NO_GPS:
    default:
        return 0;
    }
}

static uint8_t map_fix_flags(const AP_GPS::GPS_Status st)
{
    uint8_t flags = 0;

    if (st >= AP_GPS::GPS_OK_FIX_2D) {
        flags |= 1U << 0; // gnssFixOK
    }
    if (st >= AP_GPS::GPS_OK_FIX_3D_DGPS) {
        flags |= 1U << 1; // diffSoln
    }
    if (st == AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT) {
        flags |= 1U << 6; // carrSoln=float
    } else if (st == AP_GPS::GPS_OK_FIX_3D_RTK_FIXED) {
        flags |= 1U << 7; // carrSoln=fixed
    }

    return flags;
}

static uint32_t gps_itow_ms(const AP_GPS &gps, uint32_t fallback_ms)
{
    const uint32_t tow = gps.time_week_ms();
    if (tow != 0) {
        return tow;
    }
    return fallback_ms % 604800000UL;
}

static bool fake_mode_enabled()
{
    if (hal.rcin == nullptr) {
        return false;
    }

    const uint16_t pwm = hal.rcin->read(FAKE_GPS_RC_CHANNEL_INDEX);
    return pwm > FAKE_GPS_PWM_HIGH;
}

static void update_anchor_from_real_gps_if_available()
{
    AP_GPS &gps = AP::gps();
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    const Location &loc = gps.location();
    if (loc.lat == 0 || loc.lng == 0) {
        return;
    }

    g_fake_state.anchor_lat = loc.lat;
    g_fake_state.anchor_lon = loc.lng;
    g_fake_state.anchor_alt_cm = loc.alt;

    float course_deg = gps.ground_course();
    if (!isfinite(course_deg)) {
        const Vector3f &vel = gps.velocity();
        course_deg = wrap_360(degrees(atan2f(vel.y, vel.x)));
    }
    if (isfinite(course_deg)) {
        g_fake_state.heading_deg = wrap_360(course_deg);
    }
}

static void enter_fake_mode(uint32_t now_ms)
{
    g_fake_state.active = true;
    g_fake_state.initialized = true;
    g_fake_state.last_update_ms = now_ms;
    g_fake_state.fake_itow_ms = now_ms % 604800000UL;

    update_anchor_from_real_gps_if_available();

    g_fake_state.north_m = 0.0f;
    g_fake_state.east_m  = 0.0f;
    g_fake_state.down_m  = 0.0f;
    g_fake_state.speed_mps = 0.0f;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "FAKE_UBX: enter fake mode lat=%.6f lon=%.6f alt=%.2f",
                  g_fake_state.anchor_lat * 1.0e-7f,
                  g_fake_state.anchor_lon * 1.0e-7f,
                  g_fake_state.anchor_alt_cm * 0.01f);
}

static void leave_fake_mode()
{
    g_fake_state.active = false;
    g_fake_state.initialized = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FAKE_UBX: leave fake mode");
}

static void update_fake_state(uint32_t now_ms)
{
    if (!g_fake_state.initialized) {
        return;
    }

    const uint32_t dt_ms = now_ms - g_fake_state.last_update_ms;
    g_fake_state.last_update_ms = now_ms;
    g_fake_state.fake_itow_ms += dt_ms;

    if (!FAKE_SCRIPT_MOVE_ENABLE) {
        // 锁点模式：位置不变，速度为 0
        g_fake_state.speed_mps = 0.0f;
        return;
    }

    // 脚本运动模式
    g_fake_state.heading_deg = wrap_360(FAKE_SCRIPT_HEADING_DEG);
    g_fake_state.speed_mps = FAKE_SCRIPT_SPEED_MPS;

    const float dt = dt_ms * 0.001f;
    const float yaw_rad = radians(g_fake_state.heading_deg);

    g_fake_state.north_m += cosf(yaw_rad) * g_fake_state.speed_mps * dt;
    g_fake_state.east_m  += sinf(yaw_rad) * g_fake_state.speed_mps * dt;
}

static bool build_real_solution(uint32_t now_ms, NavSolution &sol)
{
    AP_GPS &gps = AP::gps();

    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        return false;
    }

    const Location &loc = gps.location();
    if (loc.lat == 0 || loc.lng == 0) {
        return false;
    }

    const Vector3f &vel = gps.velocity();

    float hacc_m = 1.5f;
    float vacc_m = 2.5f;
    if (!gps.horizontal_accuracy(hacc_m)) {
        hacc_m = 1.5f;
    }
    if (!gps.vertical_accuracy(vacc_m)) {
        vacc_m = 2.5f;
    }

    float course_deg = gps.ground_course();
    if (!isfinite(course_deg)) {
        course_deg = wrap_360(degrees(atan2f(vel.y, vel.x)));
    }
    if (!isfinite(course_deg)) {
        course_deg = 0.0f;
    }

    uint8_t sats = gps.num_sats();
    if (sats < 10) {
        sats = 10;
    }

    sol.lat = loc.lat;
    sol.lon = loc.lng;
    sol.alt_cm = loc.alt;
    sol.velN = vel.x;
    sol.velE = vel.y;
    sol.velD = vel.z;
    sol.groundspeed = gps.ground_speed();
    sol.course_deg = wrap_360(course_deg);
    sol.fixType = map_fix_type(gps.status());
    sol.flags = map_fix_flags(gps.status());
    sol.numSV = sats;
    sol.hAcc_mm = uint32_t(fmaxf(0.2f, hacc_m) * 1000.0f);
    sol.vAcc_mm = uint32_t(fmaxf(0.3f, vacc_m) * 1000.0f);
    sol.sAcc_mmps = 200;
    sol.iTOW = gps_itow_ms(gps, now_ms);

    // 持续更新 fake 模式的锚点
    g_fake_state.anchor_lat = loc.lat;
    g_fake_state.anchor_lon = loc.lng;
    g_fake_state.anchor_alt_cm = loc.alt;

    return true;
}

static bool build_fake_solution(uint32_t now_ms, NavSolution &sol)
{
    if (!g_fake_state.initialized) {
        enter_fake_mode(now_ms);
    }

    update_fake_state(now_ms);

    const double lat0_deg = g_fake_state.anchor_lat * 1.0e-7;
    const double lon0_deg = g_fake_state.anchor_lon * 1.0e-7;
    const double lat_rad = radians(lat0_deg);

    const double dlat_deg = (g_fake_state.north_m / 6378137.0) * 180.0 / M_PI;
    const double dlon_deg = (g_fake_state.east_m / (6378137.0 * fmax(0.01, cos(lat_rad)))) * 180.0 / M_PI;

    sol.lat = int32_t(lrint((lat0_deg + dlat_deg) * 1.0e7));
    sol.lon = int32_t(lrint((lon0_deg + dlon_deg) * 1.0e7));
    sol.alt_cm = g_fake_state.anchor_alt_cm - int32_t(lrint(g_fake_state.down_m * 100.0f));

    const float yaw_rad = radians(g_fake_state.heading_deg);
    sol.velN = cosf(yaw_rad) * g_fake_state.speed_mps;
    sol.velE = sinf(yaw_rad) * g_fake_state.speed_mps;
    sol.velD = 0.0f;
    sol.groundspeed = g_fake_state.speed_mps;
    sol.course_deg = wrap_360(g_fake_state.heading_deg);

    sol.fixType = 3;
    sol.flags = 0x01; // gnssFixOK
    sol.numSV = DEFAULT_FAKE_NUM_SATS;
    sol.hAcc_mm = DEFAULT_FAKE_HACC_MM;
    sol.vAcc_mm = DEFAULT_FAKE_VACC_MM;
    sol.sAcc_mmps = DEFAULT_FAKE_SACC_MMPS;
    sol.iTOW = g_fake_state.fake_itow_ms;

    return true;
}

} // namespace

GPS_SEND::GPS_SEND() :
    uart(nullptr)
{
}

void GPS_SEND::init(void)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_FAKE_GPS, 0);

    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FAKE_UBX: no UART found");
        return;
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "FAKE_UBX: UART initialized");
}

void GPS_SEND::update(void)
{
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_no_uart_ms = 0;
    static uint32_t last_no_gps_ms = 0;
    static bool last_fake_enable = false;

    if (uart == nullptr) {
        if (now_ms - last_no_uart_ms > 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FAKE_UBX: UART not ready");
            last_no_uart_ms = now_ms;
        }
        return;
    }

    process_rx();

    if (now_ms - last_send_ms < send_interval_ms) {
        return;
    }
    last_send_ms = now_ms;

    const bool fake_enable = fake_mode_enabled();

    if (fake_enable && !last_fake_enable) {
        enter_fake_mode(now_ms);
    } else if (!fake_enable && last_fake_enable) {
        leave_fake_mode();
    }
    last_fake_enable = fake_enable;

    g_nav_sol_valid = false;

    if (fake_enable) {
        g_nav_sol_valid = build_fake_solution(now_ms, g_nav_sol);
    } else {
        g_nav_sol_valid = build_real_solution(now_ms, g_nav_sol);

        if (!g_nav_sol_valid) {
            if (now_ms - last_no_gps_ms > 1000) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FAKE_UBX: source GPS no 3D fix");
                last_no_gps_ms = now_ms;
            }
            return;
        }
    }

    if (!g_nav_sol_valid) {
        return;
    }

    send_nav_packets(now_ms);
}

void GPS_SEND::process_rx(void)
{
    while (uart != nullptr && uart->available()) {
        handle_rx_byte(uart->read());
    }
}

void GPS_SEND::handle_rx_byte(uint8_t c)
{
    switch (rx.step) {
    case 0:
        rx.step = (c == UBX_SYNC1) ? 1 : 0;
        break;
    case 1:
        rx.step = (c == UBX_SYNC2) ? 2 : 0;
        break;
    case 2:
        rx.msg_class = c;
        rx.ck_a = 0;
        rx.ck_b = 0;
        checksum_update(c, rx.ck_a, rx.ck_b);
        rx.step = 3;
        break;
    case 3:
        rx.msg_id = c;
        checksum_update(c, rx.ck_a, rx.ck_b);
        rx.step = 4;
        break;
    case 4:
        rx.payload_len = c;
        checksum_update(c, rx.ck_a, rx.ck_b);
        rx.step = 5;
        break;
    case 5:
        rx.payload_len |= uint16_t(c) << 8;
        checksum_update(c, rx.ck_a, rx.ck_b);
        if (rx.payload_len > sizeof(rx.payload)) {
            rx.step = 0;
        } else {
            rx.payload_ofs = 0;
            rx.step = (rx.payload_len == 0) ? 7 : 6;
        }
        break;
    case 6:
        rx.payload[rx.payload_ofs++] = c;
        checksum_update(c, rx.ck_a, rx.ck_b);
        if (rx.payload_ofs >= rx.payload_len) {
            rx.step = 7;
        }
        break;
    case 7:
        rx.rx_ck_a = c;
        rx.step = 8;
        break;
    case 8:
        rx.rx_ck_b = c;
        if (rx.rx_ck_a == rx.ck_a && rx.rx_ck_b == rx.ck_b) {
            handle_packet(rx.msg_class, rx.msg_id, rx.payload, rx.payload_len);
        }
        rx.step = 0;
        break;
    default:
        rx.step = 0;
        break;
    }
}

void GPS_SEND::handle_packet(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    static uint32_t last_cfg_log_ms = 0;
    const uint32_t now_ms = AP_HAL::millis();

    if (now_ms - last_cfg_log_ms > 2000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "FAKE_UBX RX cls=0x%02X id=0x%02X len=%u",
                      (unsigned)msg_class,
                      (unsigned)msg_id,
                      (unsigned)len);
        last_cfg_log_ms = now_ms;
    }

    if (msg_class != UBX_CLASS_CFG) {
        return;
    }

    switch (msg_id) {
    case UBX_CFG_PRT:
        if (len == 0) {
            send_cfg_prt();
        } else {
            send_ack(msg_class, msg_id, true);
        }
        break;

    case UBX_CFG_RATE:
        if (len == 0) {
            send_cfg_rate();
        } else {
            send_ack(msg_class, msg_id, true);
        }
        break;

    case UBX_CFG_MSG:
    case UBX_CFG_NAV5:
    case UBX_CFG_VALSET:
    case UBX_CFG_VALGET:
    case UBX_CFG_VALDEL:
        send_ack(msg_class, msg_id, true);
        break;

    default:
        (void)payload;
        send_ack(msg_class, msg_id, true);
        break;
    }
}

bool GPS_SEND::send_ubx(uint8_t msg_class, uint8_t msg_id, const void *payload, uint16_t len)
{
    if (uart == nullptr) {
        return false;
    }

    uint8_t header[6];
    header[0] = UBX_SYNC1;
    header[1] = UBX_SYNC2;
    header[2] = msg_class;
    header[3] = msg_id;
    header[4] = uint8_t(len & 0xFF);
    header[5] = uint8_t((len >> 8) & 0xFF);

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (uint8_t i = 2; i < 6; i++) {
        checksum_update(header[i], ck_a, ck_b);
    }

    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(payload);
    for (uint16_t i = 0; i < len; i++) {
        checksum_update(bytes[i], ck_a, ck_b);
    }

    const size_t w0 = uart->write(header, sizeof(header));
    const size_t w1 = (len == 0) ? 0 : uart->write(bytes, len);
    const uint8_t cks[2] = {ck_a, ck_b};
    const size_t w2 = uart->write(cks, sizeof(cks));

    return (w0 == sizeof(header)) && (w1 == len) && (w2 == sizeof(cks));
}

void GPS_SEND::send_ack(uint8_t cls_id, uint8_t msg_id, bool ack)
{
    const UBXAck payload { cls_id, msg_id };
    (void)send_ubx(UBX_CLASS_ACK, ack ? UBX_ACK_ACK : UBX_ACK_NAK, &payload, sizeof(payload));
}

void GPS_SEND::send_cfg_prt(void)
{
    const UBXCfgPrt payload {
        1,          // UART1
        0,
        0,
        0x000008D0, // 8N1
        115200,
        0x0001,     // UBX input
        0x0001,     // UBX output
        0,
        0
    };
    (void)send_ubx(UBX_CLASS_CFG, UBX_CFG_PRT, &payload, sizeof(payload));
}

void GPS_SEND::send_cfg_rate(void)
{
    const UBXCfgRate payload {
        100,    // 10Hz
        1,
        1       // GPS time
    };
    (void)send_ubx(UBX_CLASS_CFG, UBX_CFG_RATE, &payload, sizeof(payload));
}

void GPS_SEND::send_nav_packets(uint32_t now_ms)
{
    if (!g_nav_sol_valid) {
        return;
    }

    const NavSolution &sol = g_nav_sol;
    const uint32_t headAcc_1e5 = DEFAULT_FAKE_HEADACC_1E5;

    UBXNavPosllh pos{};
    pos.iTOW   = sol.iTOW;
    pos.lon    = sol.lon;
    pos.lat    = sol.lat;
    pos.height = sol.alt_cm * 10; // cm -> mm
    pos.hMSL   = sol.alt_cm * 10;
    pos.hAcc   = sol.hAcc_mm;
    pos.vAcc   = sol.vAcc_mm;

    UBXNavStatus status{};
    status.iTOW   = sol.iTOW;
    status.gpsFix = sol.fixType;
    status.flags  = sol.flags;
    status.fixStat = 0;
    status.flags2  = 0;
    status.ttff    = 1000;
    status.msss    = now_ms;

    UBXNavDop dop{};
    dop.iTOW = sol.iTOW;
    dop.gDOP = 150;
    dop.pDOP = 120;
    dop.tDOP = 100;
    dop.vDOP = 150;
    dop.hDOP = 120;
    dop.nDOP = 100;
    dop.eDOP = 100;

    UBXNavVelned velned{};
    velned.iTOW = sol.iTOW;
    velned.velN = int32_t(lrintf(sol.velN * 100.0f));
    velned.velE = int32_t(lrintf(sol.velE * 100.0f));
    velned.velD = int32_t(lrintf(sol.velD * 100.0f));
    velned.speed  = uint32_t(lrintf(sqrtf(sol.velN * sol.velN + sol.velE * sol.velE + sol.velD * sol.velD) * 100.0f));
    velned.gSpeed = uint32_t(lrintf(sol.groundspeed * 100.0f));
    velned.heading = int32_t(lrintf(wrap_360(sol.course_deg) * 100000.0f));
    velned.sAcc = sol.sAcc_mmps;
    velned.cAcc = headAcc_1e5;

    UBXNavPvt pvt{};
    pvt.iTOW  = sol.iTOW;
    pvt.year  = 2026;
    pvt.month = 1;
    pvt.day   = 1;
    pvt.hour  = 0;
    pvt.min   = 0;
    pvt.sec   = 0;
    pvt.valid = 0x07;   // date/time fully valid
    pvt.tAcc  = 0;
    pvt.nano  = 0;
    pvt.fixType = sol.fixType;
    pvt.flags   = sol.flags;
    pvt.flags2  = 0;
    pvt.numSV   = sol.numSV;
    pvt.lon     = sol.lon;
    pvt.lat     = sol.lat;
    pvt.height  = sol.alt_cm * 10;
    pvt.hMSL    = sol.alt_cm * 10;
    pvt.hAcc    = sol.hAcc_mm;
    pvt.vAcc    = sol.vAcc_mm;
    pvt.velN    = velned.velN;
    pvt.velE    = velned.velE;
    pvt.velD    = velned.velD;
    pvt.gSpeed  = int32_t(velned.gSpeed);
    pvt.headMot = velned.heading;
    pvt.sAcc    = sol.sAcc_mmps;
    pvt.headAcc = headAcc_1e5;
    pvt.pDOP    = 120;
    memset(pvt.reserved1, 0, sizeof(pvt.reserved1));
    pvt.headVeh = velned.heading;
    pvt.magDec  = 0;
    pvt.magAcc  = 0;

    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_PVT,    &pvt,    sizeof(pvt));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_POSLLH, &pos,    sizeof(pos));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_STATUS, &status, sizeof(status));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_VELNED, &velned, sizeof(velned));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_DOP,    &dop,    sizeof(dop));

    static uint32_t last_print_ms = 0;
    if (now_ms - last_print_ms > 1000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "FAKE_UBX TX mode=%s lat=%.6f lon=%.6f alt=%.2f gs=%.2f crs=%.1f sats=%u",
                      g_fake_state.active ? "FAKE" : "REAL",
                      sol.lat * 1e-7f,
                      sol.lon * 1e-7f,
                      sol.alt_cm * 0.01f,
                      sol.groundspeed,
                      sol.course_deg,
                      (unsigned)sol.numSV);
        last_print_ms = now_ms;
    }
}

void GPS_SEND::checksum_update(uint8_t data, uint8_t &ck_a, uint8_t &ck_b)
{
    ck_a = uint8_t(ck_a + data);
    ck_b = uint8_t(ck_b + ck_a);
}