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

    // 添加getter方法（如果需要）
    float get_roll() const { return data_analyze_roll; }
    float get_pitch() const { return data_analyze_pitch; }
    float get_yaw() const { return data_analyze_yaw; }
    bool is_data_valid() const { return phrased; }

private:
    static MY_LINS313 *_singleton;
    AP_HAL::UARTDriver *uart;
    LINS313_AHRS _ahrs;   
    uint32_t _last_update_time_us;

#pragma pack(push, 1)
    struct PACKED LINS313
    {   
        uint8_t accx[4];    // 大端，缩放10000
        uint8_t accy[4];
        uint8_t accz[4];
        uint8_t gyro_x[4];
        uint8_t gyro_y[4];
        uint8_t gyro_z[4];
        uint8_t roll[4];
        uint8_t pitch[4];
        uint8_t yaw[4];
        int16_t temp_board; // 小端，缩放327.68
    };
#pragma pack(pop)

    struct LINS313 pack_313_update;
    
    float data_analyze_roll;
    float data_analyze_pitch;
    float data_analyze_yaw;

    float data_analyze_acc_x;    
    float data_analyze_acc_y;   
    float data_analyze_acc_z;  

    float data_analyze_gyro_x;   
    float data_analyze_gyro_y;   
    float data_analyze_gyro_z; 

    float data_analyze_temp;   
    
    
    uint16_t recv_count;    // 已接收字节数
    uint8_t sum;            // 校验和累加器
    bool phrased;           // 数据解析成功标志
};

namespace AP {
    MY_LINS313 *my_lins313();  // 避免与类名冲突
}

