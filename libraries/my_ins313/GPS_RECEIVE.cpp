#include "GPS_RECEIVE.h"

#include <GCS_MAVLink/GCS.h>


GPS_RECEIVE::GPS_RECEIVE()
{
}

void GPS_RECEIVE::init()
{
    if (!warned) {
        gcs().send_text(
            MAV_SEVERITY_INFO,
            "GPS_RECEIVE disabled: use native GPS driver on SERIALx_PROTOCOL=5"
        );
        warned = true;
    }
}

void GPS_RECEIVE::update()
{
    // UBX 外置 GPS 模式下，测试飞控不应再从同一串口读取并消费字节。
    // 该串口现在交给 ArduPilot 的 AP_GPS/AP_GPS_UBLOX 驱动。
}
