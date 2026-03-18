#include "LINS313.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Common/AP_Common.h>
#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/LogStructure.h>

#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;
#ifndef NEW_NOTHROW
#define NEW_NOTHROW new
#endif
#define LINS313_DATA_LEN 24   // 6个float * 4字节
MY_LINS313 *MY_LINS313::_singleton = nullptr;

MY_LINS313::MY_LINS313() 
    : uart(nullptr)
    , data_analyze_roll(0.0f)
    , data_analyze_pitch(0.0f)
    , data_analyze_yaw(0.0f)
    , recv_count(0)
    , sum(0)
    , phrased(false)
    ,_last_update_time_us(0)
{
    _singleton = this;
}

MY_LINS313::MY_LINS313(MY_LINS313 &_frontend, AP_HAL::UARTDriver *uart_ptr)
    : uart(uart_ptr)
    , data_analyze_roll(0.0f)
    , data_analyze_pitch(0.0f)
    , data_analyze_yaw(0.0f)
    , recv_count(0)
    , sum(0)
    , phrased(false)
{
    _singleton = this;
}

void MY_LINS313::init()
{
    auto &my_313_p = AP::serialmanager();
    uart = my_313_p.find_serial(AP_SerialManager::SerialProtocol_FAKE_GPS,0);
    if (uart == nullptr) 
    {
        gcs().send_text(MAV_SEVERITY_INFO,"LINS313 Uart Init Error");
        return;
    }
    gcs().send_text(MAV_SEVERITY_INFO,"LINS313 Uart Init Success");
}void MY_LINS313::update(void)
{
    if (uart == nullptr) return;

    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes-- > 0) {
        int16_t r = uart->read();
        if (r < 0) break;
        uint8_t c = (uint8_t)r;

        // 状态机：帧头同步
        if (recv_count == 0) {
            if (c == LINS313_HEAD0) {
                recv_count++;
                sum = 0;
            }
        } else if (recv_count == 1) {
            if (c != LINS313_HEAD1) {
                recv_count = 0;
            } else {
                recv_count++;
            }
        }
        // 接收数据区（24字节）
        else if (recv_count < 2 + 24) {  // 帧头2字节 + 24数据
            recv_buf[recv_count - 2] = c;
            sum += c;
            recv_count++;
        }
        // 接收校验和
        else if (recv_count == 2 + 24) {
            uint8_t calculated_checksum = ~(uint8_t)sum;
            if (calculated_checksum == c) 
            {
                // 解析6个浮点数（大端转主机字节序）
                data_analyze_roll        = be_to_float(&recv_buf[0]);
                data_analyze_pitch       = be_to_float(&recv_buf[4]);
                data_analyze_yaw         = be_to_float(&recv_buf[8]);
                data_analyze_roll_apm    = be_to_float(&recv_buf[12]);
                data_analyze_pitch_apm   = be_to_float(&recv_buf[16]);
                data_analyze_yaw_apm     = be_to_float(&recv_buf[20]);

                // 调试打印（可选）
                // static uint32_t last_print = 0;
                uint32_t now = AP_HAL::millis();
                // if (now - last_print > 1000) {
                //     gcs().send_text(MAV_SEVERITY_INFO, "LINS: r=%.2f p=%.2f y=%.2f",
                //         data_analyze_roll, data_analyze_pitch, data_analyze_yaw);
                //     gcs().send_text(MAV_SEVERITY_INFO, "APM_from_send: r=%.2f p=%.2f y=%.2f",
                //         data_analyze_roll_apm, data_analyze_pitch_apm, data_analyze_yaw_apm);
                //     last_print = now;
                // }

                phrased = true;
                _ahrs.set_lins_reference(data_analyze_roll, data_analyze_pitch, data_analyze_yaw);
                _ahrs.set_lins_reference_apm_no_gps(data_analyze_roll_apm, data_analyze_pitch_apm, data_analyze_yaw_apm);
                _ahrs.log_comparison(now);

            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "LINS313 Checksum Error");
                phrased = false;
            }
            recv_count = 0; // 重置状态机
        }
    }
}
float MY_LINS313::be_to_float(const uint8_t bytes[4]) const
{
    union {
        uint32_t i;
        float f;
    } u;
    u.i = ((uint32_t)bytes[0] << 24) |
          ((uint32_t)bytes[1] << 16) |
          ((uint32_t)bytes[2] << 8)  |
          bytes[3];
    return u.f;
}
namespace AP {
    MY_LINS313 *my_lins313()
    {
        return MY_LINS313::get_singleton();
    }
}