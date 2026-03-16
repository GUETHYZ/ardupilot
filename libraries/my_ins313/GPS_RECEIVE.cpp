#include "GPS_RECEIVE.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <cmath>

extern const AP_HAL::HAL& hal;

GPS_RECEIVE::GPS_RECEIVE()
{
}

void GPS_RECEIVE::init()
{
    auto &sm = AP::serialmanager();

    uart = sm.find_serial(AP_SerialManager::SerialProtocol_LINS313_SEND, 0);

    if (uart == nullptr)
    {
        gcs().send_text(MAV_SEVERITY_ERROR, "GPS_RECEIVE UART FAIL");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "GPS_RECEIVE UART OK");
}

float GPS_RECEIVE::be_bytes_to_float(const uint8_t bytes[4]) const
{
    union
    {
        uint32_t i;
        float f;
    } u;

    u.i =
        ((uint32_t)bytes[0] << 24) |
        ((uint32_t)bytes[1] << 16) |
        ((uint32_t)bytes[2] << 8)  |
        ((uint32_t)bytes[3]);

    return u.f;
}

void GPS_RECEIVE::update()
{
    if (!uart) {
        return;
    }

    static uint32_t last_invalid_ms = 0;
    static uint32_t last_origin_wait_ms = 0;
    static uint32_t last_origin_ready_ms = 0;
    static uint32_t last_vo_debug_ms = 0;
    static uint32_t last_no_vo_ms = 0;

    while (uart->available())
    {
        const uint8_t c = uart->read();

        switch (recv_count)
        {
        case 0:
            if (c == LINS313_HEAD0) {
                recv_count = 1;
            }
            break;

        case 1:
            if (c == LINS313_HEAD1)
            {
                recv_count = 2;
                sum = 0;
            }
            else
            {
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
            else if (recv_count == 30)
            {
                const uint8_t checksum = (uint8_t)(~sum);

                if (checksum == c)
                {
                    // 新协议解析：第1个float不再是时间，而是 yaw(rad)
                    yaw  = be_bytes_to_float(&pack.data[0]);
                    lat  = be_bytes_to_float(&pack.data[4]);
                    lon  = be_bytes_to_float(&pack.data[8]);
                    alt  = be_bytes_to_float(&pack.data[12]);

                    velN = be_bytes_to_float(&pack.data[16]);
                    velE = be_bytes_to_float(&pack.data[20]);
                    velD = be_bytes_to_float(&pack.data[24]);

                    const uint32_t now_ms = AP_HAL::millis();
                    const uint64_t now_us = AP_HAL::micros64();

                    const bool valid_fix =
                        std::isfinite(yaw)  &&
                        std::isfinite(lat)  &&
                        std::isfinite(lon)  &&
                        std::isfinite(alt)  &&
                        std::isfinite(velN) &&
                        std::isfinite(velE) &&
                        std::isfinite(velD) &&
                        (fabsf(lat) > 1.0e-6f) &&
                        (fabsf(lon) > 1.0e-6f) &&
                        (fabsf(lat) <= 90.0f) &&
                        (fabsf(lon) <= 180.0f) &&
                        (fabsf(alt) < 50000.0f);

                    if (!valid_fix)
                    {
                        if (now_ms - last_invalid_ms > 1000)
                        {
                            gcs().send_text(
                                MAV_SEVERITY_WARNING,
                                "GPS INPUT INVALID lat=%.6f lon=%.6f alt=%.2f",
                                lat, lon, alt
                            );
                            last_invalid_ms = now_ms;
                        }

                        recv_count = 0;
                        continue;
                    }

                    AP_AHRS &ahrs = AP::ahrs();

                    // --------------------------------------------------
                    // 1) 优先看看系统内部是否已经有 origin
                    // --------------------------------------------------
                    Location ekf_origin;
                    if (ahrs.get_origin(ekf_origin))
                    {
                        origin = ekf_origin;

                        if (!origin_set && now_ms - last_origin_ready_ms > 1000)
                        {
                            gcs().send_text(
                                MAV_SEVERITY_INFO,
                                "USE EKF ORIGIN %.6f %.6f",
                                origin.lat * 1e-7,
                                origin.lng * 1e-7
                            );
                            last_origin_ready_ms = now_ms;
                        }

                        origin_set = true;
                    }
                    else
                    {
                        // --------------------------------------------------
                        // 2) 系统还没有 origin，则自己尝试设置
                        // --------------------------------------------------
                        if (origin_count < 3)
                        {
                            origin_lat_sum += lat;
                            origin_lon_sum += lon;
                            origin_alt_sum += alt;
                            origin_count++;
                        }

                        if (origin_count >= 3)
                        {
                            Location candidate_origin;
                            candidate_origin.lat = (int32_t)lrintf((origin_lat_sum / origin_count) * 1e7f);
                            candidate_origin.lng = (int32_t)lrintf((origin_lon_sum / origin_count) * 1e7f);
                            candidate_origin.alt = (int32_t)lrintf((origin_alt_sum / origin_count) * 100.0f);

                            if (ahrs.set_origin(candidate_origin))
                            {
                                origin = candidate_origin;
                                origin_set = true;

                                gcs().send_text(
                                    MAV_SEVERITY_INFO,
                                    "EKF ORIGIN SET %.6f %.6f",
                                    origin.lat * 1e-7,
                                    origin.lng * 1e-7
                                );
                            }
                            else
                            {
                                // set_origin失败后，再检查一次系统内部是否已经有origin
                                if (ahrs.get_origin(ekf_origin))
                                {
                                    origin = ekf_origin;
                                    origin_set = true;

                                    if (now_ms - last_origin_ready_ms > 1000)
                                    {
                                        gcs().send_text(
                                            MAV_SEVERITY_INFO,
                                            "USE EKF ORIGIN %.6f %.6f",
                                            origin.lat * 1e-7,
                                            origin.lng * 1e-7
                                        );
                                        last_origin_ready_ms = now_ms;
                                    }
                                }
                                else
                                {
                                    origin_set = false;

                                    if (now_ms - last_origin_wait_ms > 1000)
                                    {
                                        gcs().send_text(
                                            MAV_SEVERITY_WARNING,
                                            "WAIT EKF ORIGIN"
                                        );
                                        last_origin_wait_ms = now_ms;
                                    }
                                }
                            }
                        }
                    }

                    // --------------------------------------------------
                    // 3) 只有真正拿到origin，才进行VO注入
                    // --------------------------------------------------
                    if (origin_set)
                    {
                        Location loc;
                        loc.lat = (int32_t)lrintf(lat * 1e7f);
                        loc.lng = (int32_t)lrintf(lon * 1e7f);
                        loc.alt = (int32_t)lrintf(alt * 100.0f);

                        const Vector3f posNED = origin.get_distance_NED(loc);

                        AP_VisualOdom *vo = AP::visualodom();

                        if (vo != nullptr)
                        {
                            const float yaw_rad = wrap_PI(yaw);
                            const Vector3f vel(velN, velE, velD);

                            vo->handle_pose_estimate(
                                now_us,
                                now_ms,

                                posNED.x,
                                posNED.y,
                                posNED.z,

                                0.0f,
                                0.0f,
                                yaw_rad,     // 这里改为主飞控传来的 yaw(rad)

                                0.5f,
                                0.1f,

                                0,
                                100
                            );

                            vo->handle_vision_speed_estimate(
                                now_us,
                                now_ms,
                                vel,
                                0,
                                100
                            );

                            if (now_ms - last_vo_debug_ms > 1000)
                            {
                                gcs().send_text(
                                    MAV_SEVERITY_INFO,
                                    "VO NED %.2f %.2f %.2f YAW %.1f",
                                    posNED.x,
                                    posNED.y,
                                    posNED.z,
                                    degrees(yaw_rad)
                                );
                                last_vo_debug_ms = now_ms;
                            }
                        }
                        else
                        {
                            if (now_ms - last_no_vo_ms > 1000)
                            {
                                gcs().send_text(
                                    MAV_SEVERITY_WARNING,
                                    "VISUALODOM NULL"
                                );
                                last_no_vo_ms = now_ms;
                            }
                        }
                    }
                }

                recv_count = 0;
            }
            break;
        }
    }
}