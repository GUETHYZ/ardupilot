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

    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_ERROR, "GPS_RECEIVE UART FAIL");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "GPS_RECEIVE UART OK");
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

uint32_t GPS_RECEIVE::be_bytes_to_u32(const uint8_t bytes[4]) const
{
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8)  |
           ((uint32_t)bytes[3]);
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
            if (recv_count < 30) {
                pack.data[recv_count - 2] = c;
                sum += c;
                recv_count++;
            } else {
                const uint8_t checksum = ~sum;
                if (checksum == c) {
                    // 测试飞控回传时间戳（ms）
                    remote_boot_ms = be_bytes_to_u32(&pack.data[0]);

                    // 测试飞控回传的位置与速度
                    remote_lat_deg   = be_bytes_to_float(&pack.data[4]);
                    remote_lon_deg   = be_bytes_to_float(&pack.data[8]);
                    remote_alt_m     = be_bytes_to_float(&pack.data[12]);
                    remote_vel_n_mps = be_bytes_to_float(&pack.data[16]);
                    remote_vel_e_mps = be_bytes_to_float(&pack.data[20]);
                    remote_vel_d_mps = be_bytes_to_float(&pack.data[24]);

                    const uint32_t now_ms = AP_HAL::millis();
                    static uint32_t last_print_ms = 0;

                    if (now_ms - last_print_ms >= 1000) 
                    {
                        gcs().send_text(
                            MAV_SEVERITY_INFO,
                            "REMOTE_POS t=%lu lat=%.7f lon=%.7f alt=%.2f",
                            (unsigned long)remote_boot_ms,
                            remote_lat_deg,
                            remote_lon_deg,
                            remote_alt_m);

                        gcs().send_text(
                            MAV_SEVERITY_INFO,
                            "REMOTE_VEL vN=%.2f vE=%.2f vD=%.2f",
                            remote_vel_n_mps,
                            remote_vel_e_mps,
                            remote_vel_d_mps);

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

    Location remote_loc;
    remote_loc.lat = (int32_t)lrintf(remote_lat_deg * 1e7f);
    remote_loc.lng = (int32_t)lrintf(remote_lon_deg * 1e7f);
    remote_loc.alt = (int32_t)lrintf(remote_alt_m * 100.0f);   // m -> cm

    static bool compare_ref_set = false;
    static Location compare_ref_loc;

    if (!compare_ref_set) {
        compare_ref_loc = remote_loc;
        compare_ref_set = true;
    }

    // 统一转换到同一个参考点下的 NED（单位：米）
    const Vector3f remote_ned_m = compare_ref_loc.get_distance_NED(remote_loc);
    const Vector3f main_ned_m   = compare_ref_loc.get_distance_NED(main_loc);
    const Vector3f pos_err_ned_m = remote_ned_m - main_ned_m;

    Vector3f main_vel_ned_mps;
    if (!ahrs.get_velocity_NED(main_vel_ned_mps)) {
        main_vel_ned_mps.zero();
    }

    const Vector3f remote_vel_ned_mps(
        remote_vel_n_mps,
        remote_vel_e_mps,
        remote_vel_d_mps);

    const Vector3f vel_err_ned_mps = remote_vel_ned_mps - main_vel_ned_mps;



//     struct PACKED log_GPEK
// {
//     LOG_PACKET_HEADER;
//     uint64_t time_us;

//     float remote_n;
//     float remote_e;
//     float remote_d;

//     float main_n;
//     float main_e;
//     float main_d;

//     float pos_err_n;
//     float pos_err_e;
//     float pos_err_d;

//     float vel_err_n;
//     float vel_err_e;
//     float vel_err_d;
// };
    const struct log_GPEK pkt = {
        LOG_PACKET_HEADER_INIT(LOG_GPEK_MSG),
        time_us:time_us,

        remote_n:remote_ned_m.x, remote_e:remote_ned_m.y, remote_d:remote_ned_m.z,
        main_n:main_ned_m.x,   main_e:main_ned_m.y,   main_d:main_ned_m.z,

        pos_err_n:pos_err_ned_m.x, pos_err_e:pos_err_ned_m.y, pos_err_d:pos_err_ned_m.z,
        vel_err_n:vel_err_ned_mps.x, vel_err_e:vel_err_ned_mps.y, vel_err_d:vel_err_ned_mps.z
    };

    AP::logger().WriteBlock(&pkt, sizeof(pkt));
#endif
}