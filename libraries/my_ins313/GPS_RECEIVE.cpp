#include "GPS_RECEIVE.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>

extern const AP_HAL::HAL& hal;

GPS_RECEIVE::GPS_RECEIVE()
{
}

void GPS_RECEIVE::init()
{
    auto &sm = AP::serialmanager();

    uart = sm.find_serial(AP_SerialManager::SerialProtocol_MESSAGE_RT, 0);

    if (uart == nullptr)
    {
        gcs().send_text(MAV_SEVERITY_ERROR,"GPS_RECEIVE UART FAIL");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO,"GPS_RECEIVE UART OK");
}



#define GPSRET_HEAD0 0xAA
#define GPSRET_HEAD1 0x55

float GPS_RECEIVE::be_bytes_to_float(const uint8_t bytes[4]) const
{
    union {
        uint32_t i;
        float f;
    } u;

    u.i = ((uint32_t)bytes[0] << 24) |
          ((uint32_t)bytes[1] << 16) |
          ((uint32_t)bytes[2] << 8)  |
          ((uint32_t)bytes[3]);
    return u.f;
}

void GPS_RECEIVE::update(void)
{
    if (uart == nullptr) {
        return;
    }

    while (uart->available()) {
        const uint8_t c = uart->read();

        switch (recv_count) {
        case 0:
            recv_count = (c == GPSRET_HEAD0) ? 1 : 0;
            break;

        case 1:
            if (c == GPSRET_HEAD1) {
                recv_count = 2;
                sum = 0;
            } else {
                recv_count = 0;
            }
            break;

        default:
            if (recv_count < 30) 
            {
                pack.data[recv_count - 2] = c;
                sum += c;
                recv_count++;
            } 
            else 
            {
                const uint8_t checksum = ~sum;
                if (checksum == c) {
                    // pack.data[0..3] 是测试飞控时间戳，这里先不使用
                    lat  = be_bytes_to_float(&pack.data[4]);
                    lon  = be_bytes_to_float(&pack.data[8]);
                    alt  = be_bytes_to_float(&pack.data[12]);
                    velN = be_bytes_to_float(&pack.data[16]);
                    velE = be_bytes_to_float(&pack.data[20]);
                    velD = be_bytes_to_float(&pack.data[24]);
                    static uint32_t last_print_ms = 0;
                    uint32_t now_ms = AP_HAL::millis();
                    if (now_ms - last_print_ms >= 1000) {  // 每秒打印一次
                        //gcs().send_text(MAV_SEVERITY_INFO, "GPS_RECEIVE: lat=%.7f lon=%.7f alt=%.2f", lat, lon, alt);
                        //gcs().send_text(MAV_SEVERITY_INFO, "GPS_RECEIVE: velN=%.2f velE=%.2f velD=%.2f", velN, velE, velD);
                        last_print_ms = now_ms;
                    }
                    log_compare(AP_HAL::micros64());
                }
                recv_count = 0;
            }
            break;
        }
    }
}



void GPS_RECEIVE::log_compare(uint64_t time_us)
{
#if HAL_LOGGING_ENABLED
    AP_AHRS &ahrs = AP::ahrs();

    Location main_loc;
    if (!ahrs.get_location(main_loc)) {
        return;
    }

    Location test_loc;
    test_loc.lat = (int32_t)lrintf(lat * 1e7f);
    test_loc.lng = (int32_t)lrintf(lon * 1e7f);
    test_loc.alt = (int32_t)lrintf(alt * 100.0f);

    static bool cmp_origin_set = false;
    static Location cmp_origin;

    if (!cmp_origin_set) {
        cmp_origin = test_loc;
        cmp_origin_set = true;
    }

    const Vector3f test_ned = cmp_origin.get_distance_NED(test_loc);
    const Vector3f main_ned = cmp_origin.get_distance_NED(main_loc);
    const Vector3f dpos = test_ned - main_ned;

    Vector3f main_vel;
    if (!ahrs.get_velocity_NED(main_vel)) {
        main_vel.zero();
    }

    const Vector3f test_vel(velN, velE, velD);
    const Vector3f dvel = test_vel - main_vel;

    const struct log_GPEK pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPEK_MSG),
        time_us,
        test_ned.x, test_ned.y, test_ned.z,
        main_ned.x, main_ned.y, main_ned.z,
        dpos.x, dpos.y, dpos.z,
        dvel.x, dvel.y, dvel.z
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}