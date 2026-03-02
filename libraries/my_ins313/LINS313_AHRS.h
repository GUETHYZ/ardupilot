#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/definitions.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

class LINS313_AHRS {
public:
    LINS313_AHRS();
    

    
    // 设置LINS313直接输出的姿态（用于对比）
    void set_lins_reference(float roll_deg, float pitch_deg, float yaw_deg);
    void set_lins_reference_apm_no_gps(float roll_deg_no_gps, float pitch_deg_no_gps, float yaw_deg_no_gps);
    // 对比数据获取
    struct ComparisonData {
        uint64_t time_us;
        float lins_roll_deg;
        float lins_pitch_deg;
        float lins_yaw_deg;
        float dcm_roll_deg;
        float dcm_pitch_deg;
        float dcm_yaw_deg;
        float apm_roll_deg;
        float apm_pitch_deg;
        float apm_yaw_deg;
        float error_rp;
        float error_yaw;
    };
    
    void get_comparison_data(ComparisonData &data);
    void log_comparison(uint64_t time_us);
    
private:
    float _adaptive_kp;
    float _adaptive_ki;
    uint32_t _last_static_time;
    bool _was_dynamic;
    

    
    // 欧拉角输出
    float _roll_rad;
    float _pitch_rad;
    float _yaw_rad;

    
    // 外部参考数据（用于对比）
    float _lins_roll_deg;
    float _lins_pitch_deg;
    float _lins_yaw_deg;
};