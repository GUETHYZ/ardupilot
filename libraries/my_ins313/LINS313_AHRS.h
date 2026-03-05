#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/definitions.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

extern float roll_from_apm_dcm_no_gps, pitch_from_apm_dcm_no_gps, yaw_from_apm_dcm_no_gps;

class LINS313_AHRS {
public:
    LINS313_AHRS();
    
    // 使用LINS313的加速度和陀螺仪数据进行DCM解算
    void update(const Vector3f &gyro_rads, const Vector3f &accel_mss, float dt);
    
    // 设置LINS313直接输出的姿态（用于对比）
    void set_lins_reference(float roll_deg, float pitch_deg, float yaw_deg);
    
    // 获取DCM解算结果
    void get_euler_angles(float &roll_rad, float &pitch_rad, float &yaw_rad) const;
    
    // 获取DCM矩阵
    const Matrix3f& get_dcm_matrix() const { return _dcm_matrix; }
    
    // 获取角速度估计
    const Vector3f& get_gyro_estimate() const { return _omega; }
    
    // 状态查询
    bool is_initialized() const { return _initialized; }
    bool is_healthy() const { return _healthy; }
    float get_error_rp() const { return _error_rp; }
    float get_error_yaw() const { return _error_yaw; }
    
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
    
    void get_comparison_data(ComparisonData &data) const;
    void log_comparison(uint64_t time_us);
    
private:
    float _adaptive_kp;
    float _adaptive_ki;
    uint32_t _last_static_time;
    bool _was_dynamic;
    
    Vector3f _velocity_estimate;
    Vector3f _last_velocity_estimate;
    Vector3f _filtered_motion_accel;
    

    // DCM核心算法（与AP_AHRS_DCM保持一致）
    void matrix_update(float dt);
    void normalize();
    void drift_correction(float dt);
    void drift_correction_yaw();
    bool renorm(const Vector3f &a, Vector3f &result);
    float _P_gain(float spin_rate);
    float _yaw_gain() const;
    void check_matrix();
    
    // 成员变量
    Matrix3f _dcm_matrix;          // DCM矩阵
    Matrix3f _body_dcm_matrix;     // 机体坐标系DCM
    Vector3f _omega;               // 估计的角速度 (rad/s)
    Vector3f _omega_P;             // 比例修正项
    Vector3f _omega_I;             // 积分修正项
    Vector3f _omega_yaw_P;         // 偏航比例修正项
    
    // 传感器数据
    Vector3f _gyro_accum;          // 陀螺仪数据 (rad/s)
    Vector3f _accel_accum;         // 加速度计数据 (m/s²)
    
    // 欧拉角输出
    float _roll_rad;
    float _pitch_rad;
    float _yaw_rad;
    
    // 三角函数缓存
    float _cos_yaw;
    float _sin_yaw;
    
    // 状态标志
    bool _initialized;
    bool _healthy;
    uint32_t _last_update_ms;
    
    // 误差统计
    float _error_rp;
    float _error_yaw;
    
    // 漂移校正相关
    Vector3f _ra_sum;
    float _ra_deltat;
    
    // DCM参数 (与AP_AHRS_DCM保持一致)
    static constexpr float _kp = 0.95f;          // 比例增益
    static constexpr float _ki = 0.0087f;        // 积分增益
    static constexpr float _kp_yaw = 0.5f;       // 偏航比例增益
    static constexpr float SPIN_RATE_LIMIT = 20.0f; // 度/秒

    
    // 外部参考数据（用于对比）
    float _lins_roll_deg;
    float _lins_pitch_deg;
    float _lins_yaw_deg;
};