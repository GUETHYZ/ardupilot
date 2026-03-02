#pragma once

#include <AP_MSP/msp.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>
#include <Filter/Filter.h>
#include <Filter/ModeFilter.h>
#include "LINS313_AHRS.h"

#define LINS313_HEAD0  0x7F
#define LINS313_HEAD1  0x94

#ifndef MY_LINS313_ENABLED
#define MY_LINS313_ENABLED 1
#endif

class MY_LINS313
{
public:
    MY_LINS313();
    MY_LINS313(MY_LINS313 &_frontend, AP_HAL::UARTDriver *uart);
    
    // get singleton instance
    static MY_LINS313 *get_singleton() {
        return _singleton;
    }

    void update(void);
    void init(void);
    static MY_LINS313 *detect(MY_LINS313 &_frontend);

    // getter methods
    float get_roll() const { return data_analyze_roll; }
    float get_pitch() const { return data_analyze_pitch; }
    float get_yaw() const { return data_analyze_yaw; }
    float get_apm_roll() const { return data_analyze_roll_apm; }
    float get_apm_pitch() const { return data_analyze_pitch_apm; }
    float get_apm_yaw() const { return data_analyze_yaw_apm; }
    bool is_data_valid() const { return phrased; }

    // 获取对比数据（供发送端使用）
    bool get_comparison_data(LINS313_AHRS::ComparisonData &data) const;

private:
    static MY_LINS313 *_singleton;
    AP_HAL::UARTDriver *uart;
    LINS313_AHRS _ahrs;   
    uint32_t _last_update_time_us;

    // 接收缓冲区：存储24字节数据区
    uint8_t recv_buf[24];
    
    // 解析后的数据（与发送协议顺序一致）
    float data_analyze_roll;          // LINS roll (来自发送端)
    float data_analyze_pitch;         // LINS pitch
    float data_analyze_yaw;           // LINS yaw
    float data_analyze_roll_apm;      // APM roll (来自发送端)
    float data_analyze_pitch_apm;     // APM pitch
    float data_analyze_yaw_apm;       // APM yaw

    // 状态机变量
    uint16_t recv_count;    // 已接收字节数
    uint8_t sum;            // 校验和累加器
    bool phrased;           // 数据解析成功标志

    // 辅助函数：将4字节大端数据转换为float
    float be_to_float(const uint8_t bytes[4]) const;
};

namespace AP {
    MY_LINS313 *my_lins313();  // 避免与类名冲突
}