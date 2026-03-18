#include "GPS_SEND.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
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

static constexpr uint8_t UBX_CFG_PRT  = 0x00;
static constexpr uint8_t UBX_CFG_MSG  = 0x01;
static constexpr uint8_t UBX_CFG_RATE = 0x08;
static constexpr uint8_t UBX_CFG_NAV5 = 0x24;
static constexpr uint8_t UBX_CFG_VALSET = 0x8A;
static constexpr uint8_t UBX_CFG_VALGET = 0x8B;
static constexpr uint8_t UBX_CFG_VALDEL = 0x8C;

#pragma pack(push, 1)
struct UBXNavPosllh {
    uint32_t iTOW;-
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
        flags |= 1U << 6; // carrSoln = float (01)
    } else if (st == AP_GPS::GPS_OK_FIX_3D_RTK_FIXED) {
        flags |= 1U << 7; // carrSoln = fixed (10)
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

    AP_GPS &gps = AP::gps();
    if (gps.status() < AP_GPS::GPS_OK_FIX_3D) {
        if (now_ms - last_no_gps_ms > 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FAKE_UBX: source GPS no 3D fix");
            last_no_gps_ms = now_ms;
        }
        return;
    }

    const Location &loc = gps.location();
    if (loc.lat == 0 || loc.lng == 0) {
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
    AP_GPS &gps = AP::gps();
    const Location &loc = gps.location();
    const Vector3f &vel = gps.velocity();

    float hacc_m = 1.5f;
    float vacc_m = 2.5f;
    if (!gps.horizontal_accuracy(hacc_m)) {
        hacc_m = 1.5f;
    }
    if (!gps.vertical_accuracy(vacc_m)) {
        vacc_m = 2.5f;
    }

    const float groundspeed = gps.ground_speed();
    float course_deg = gps.ground_course();
    if (!isfinite(course_deg)) {
        course_deg = wrap_360(degrees(atan2f(vel.y, vel.x)));
    }

    const uint32_t iTOW = gps_itow_ms(gps, now_ms);
    const uint8_t fixType = map_fix_type(gps.status());
    const uint8_t flags = map_fix_flags(gps.status());
    const uint8_t sats = MAX(uint8_t(10), gps.num_sats());
    const uint32_t hAcc_mm = uint32_t(MAX(0.2f, hacc_m) * 1000.0f);
    const uint32_t vAcc_mm = uint32_t(MAX(0.3f, vacc_m) * 1000.0f);
    const uint32_t sAcc_mmps = 200;
    const uint32_t headAcc_1e5 = 500000; // 5 deg

    UBXNavPosllh pos{};
    pos.iTOW = iTOW;
    pos.lon = loc.lng;
    pos.lat = loc.lat;
    pos.height = loc.alt * 10; // cm -> mm
    pos.hMSL = loc.alt * 10;
    pos.hAcc = hAcc_mm;
    pos.vAcc = vAcc_mm;

    UBXNavStatus status{};
    status.iTOW = iTOW;
    status.gpsFix = fixType;
    status.flags = flags;
    status.fixStat = 0;
    status.flags2 = 0;
    status.ttff = 1000;
    status.msss = now_ms;

    UBXNavDop dop{};
    dop.iTOW = iTOW;
    dop.gDOP = 150;
    dop.pDOP = 120;
    dop.tDOP = 100;
    dop.vDOP = 150;
    dop.hDOP = 120;
    dop.nDOP = 100;
    dop.eDOP = 100;

    UBXNavVelned velned{};
    velned.iTOW = iTOW;
    velned.velN = int32_t(lrintf(vel.x * 100.0f));
    velned.velE = int32_t(lrintf(vel.y * 100.0f));
    velned.velD = int32_t(lrintf(vel.z * 100.0f));
    velned.speed = uint32_t(lrintf(sqrtf(vel.x * vel.x + vel.y * vel.y + vel.z * vel.z) * 100.0f));
    velned.gSpeed = uint32_t(lrintf(groundspeed * 100.0f));
    velned.heading = int32_t(lrintf(course_deg * 100000.0f));
    velned.sAcc = sAcc_mmps;
    velned.cAcc = headAcc_1e5;

    UBXNavPvt pvt{};
    pvt.iTOW = iTOW;
    pvt.year = 2026;
    pvt.month = 1;
    pvt.day = 1;
    pvt.hour = 0;
    pvt.min = 0;
    pvt.sec = 0;
    pvt.valid = 0;
    pvt.tAcc = 0;
    pvt.nano = 0;
    pvt.fixType = fixType;
    pvt.flags = flags;
    pvt.flags2 = 0;
    pvt.numSV = sats;
    pvt.lon = loc.lng;
    pvt.lat = loc.lat;
    pvt.height = loc.alt * 10;
    pvt.hMSL = loc.alt * 10;
    pvt.hAcc = hAcc_mm;
    pvt.vAcc = vAcc_mm;
    pvt.velN = velned.velN;
    pvt.velE = velned.velE;
    pvt.velD = velned.velD;
    pvt.gSpeed = int32_t(velned.gSpeed);
    pvt.headMot = velned.heading;
    pvt.sAcc = sAcc_mmps;
    pvt.headAcc = headAcc_1e5;
    pvt.pDOP = 120;
    memset(pvt.reserved1, 0, sizeof(pvt.reserved1));
    pvt.headVeh = velned.heading;
    pvt.magDec = 0;
    pvt.magAcc = 0;

    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_PVT, &pvt, sizeof(pvt));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_POSLLH, &pos, sizeof(pos));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_STATUS, &status, sizeof(status));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_VELNED, &velned, sizeof(velned));
    (void)send_ubx(UBX_CLASS_NAV, UBX_NAV_DOP, &dop, sizeof(dop));

    static uint32_t last_print_ms = 0;
    if (now_ms - last_print_ms > 1000) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "FAKE_UBX TX lat=%.6f lon=%.6f alt=%.2f gs=%.2f crs=%.1f sats=%u",
                      loc.lat * 1e-7f,
                      loc.lng * 1e-7f,
                      loc.alt * 0.01f,
                      groundspeed,
                      course_deg,
                      (unsigned)sats);
        last_print_ms = now_ms;
    }
}

void GPS_SEND::checksum_update(uint8_t data, uint8_t &ck_a, uint8_t &ck_b)
{
    ck_a = uint8_t(ck_a + data);
    ck_b = uint8_t(ck_b + ck_a);
}
