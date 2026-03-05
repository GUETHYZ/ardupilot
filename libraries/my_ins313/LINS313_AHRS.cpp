#include "LINS313_AHRS.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/definitions.h>
#include <AP_AHRS/AP_AHRS_DCM.h>

extern const AP_HAL::HAL& hal;

float roll_from_apm_dcm_no_gps, pitch_from_apm_dcm_no_gps, yaw_from_apm_dcm_no_gps;

LINS313_AHRS::LINS313_AHRS() :
    _initialized(false),
    _healthy(false),
    _last_update_ms(0),
    _error_rp(1.0f),
    _error_yaw(1.0f),
    _ra_deltat(0),
    _lins_roll_deg(0),
    _lins_pitch_deg(0),
    _lins_yaw_deg(0)
{
    // 初始化DCM矩阵为单位矩阵
    _dcm_matrix.identity();
    _body_dcm_matrix.identity();
    
    _roll_rad = _pitch_rad = _yaw_rad = 0;
    _cos_yaw = 1.0f;
    _sin_yaw = 0.0f;
    
    _omega_P.zero();
    _omega_I.zero();
    _omega_yaw_P.zero();
    _omega.zero();
    _gyro_accum.zero();
    _accel_accum.zero();
    _ra_sum.zero();
}

void LINS313_AHRS::set_lins_reference(float roll_deg, float pitch_deg, float yaw_deg)
{
    _lins_roll_deg = roll_deg;
    _lins_pitch_deg = pitch_deg;
    _lins_yaw_deg = yaw_deg;
}

void LINS313_AHRS::update(const Vector3f &gyro_rads, const Vector3f &accel_mss, float dt)
{
    if (dt <= 0 || dt > 0.2f) {
        return;  
    }
    
    // 简化版运动加速度估计
    float accel_length = accel_mss.length();
    bool accel_trustworthy = (fabsf(accel_length - GRAVITY_MSS) < 3.0f);  // 静止或匀速运动
    
    // 如果加速度计可信，使用它；否则主要依赖陀螺仪
    Vector3f gravity_accel;
    if (accel_trustworthy) 
    {
        // 加速度计读数可信，直接使用
        gravity_accel = accel_mss;
    } 
    else 
    {
        // 运动状态，从加速度计中去除估计的离心加速度
        // 简单的估计：假设运动加速度与速度方向相关
        static Vector3f estimated_motion_accel;
        
        // 更新速度估计（简单的积分）
        _velocity_estimate += accel_mss * dt;
        
        // 低通滤波得到估计的运动加速度
        Vector3f motion_accel = (_velocity_estimate - _last_velocity_estimate) / dt;
        _filtered_motion_accel = _filtered_motion_accel * 0.9f + motion_accel * 0.1f;
        
        // 从加速度计读数中减去估计的运动加速度
        gravity_accel = accel_mss - _filtered_motion_accel;
        
        // 限制重力加速度的大小
        if (gravity_accel.length() > 15.0f) 
        {
            gravity_accel.normalize();
            gravity_accel *= GRAVITY_MSS;
        } 
        
        _last_velocity_estimate = _velocity_estimate;
    }
    
    // 存储传感器数据
    _gyro_accum = gyro_rads;
    _accel_accum = gravity_accel;  // 使用修正后的加速度
    
    // 初始化检查
    if (!_initialized) {
        if (gravity_accel.length() > 5.0f) 
        {
            // 使用加速度计计算初始横滚和俯仰
            // 注意坐标系：LINS313的Z向上，所以重力是正的
            _pitch_rad = atan2f(gravity_accel.x, norm(gravity_accel.y, gravity_accel.z));
            _roll_rad = atan2f(-gravity_accel.y, -gravity_accel.z);
            _yaw_rad = 0;  // 相对偏航从0开始
            
            _dcm_matrix.from_euler(_roll_rad, _pitch_rad, _yaw_rad);
            _initialized = true;
            _healthy = true;
            
            // 初始化自适应增益
            _adaptive_kp = _kp;
            _adaptive_ki = _ki;
            _was_dynamic = false;
            _last_static_time = AP_HAL::millis();
        }
        return;
    }
    
    // 执行DCM更新流程
    matrix_update(dt);
    normalize();
    drift_correction(dt);
    check_matrix();
    
    // 计算欧拉角
    _body_dcm_matrix = _dcm_matrix;
    _body_dcm_matrix.to_euler(&_roll_rad, &_pitch_rad, &_yaw_rad);
    
    // 预计算三角函数
    _cos_yaw = cosf(_yaw_rad);
    _sin_yaw = sinf(_yaw_rad);
    
    _last_update_ms = AP_HAL::millis();
}

void LINS313_AHRS::matrix_update(float dt)
{
    if (dt <= 0) return;
    
    // 角速度估计 = 原始陀螺仪 + 积分修正项
    _omega = _gyro_accum + _omega_I;
    
    // 总修正 = 角速度 + 积分项 + 比例项 + 偏航比例项
    Vector3f omega_corrected = _omega + _omega_P + _omega_yaw_P;
    
    // 构造旋转矩阵（与AP_AHRS_DCM保持一致）
    Matrix3f temp_matrix;
    temp_matrix.a.x = 0;
    temp_matrix.a.y = -omega_corrected.z * dt;
    temp_matrix.a.z = omega_corrected.y * dt;
    
    temp_matrix.b.x = omega_corrected.z * dt;
    temp_matrix.b.y = 0;
    temp_matrix.b.z = -omega_corrected.x * dt;
    
    temp_matrix.c.x = -omega_corrected.y * dt;
    temp_matrix.c.y = omega_corrected.x * dt;
    temp_matrix.c.z = 0;
    
    // 应用旋转：R_new = R_old * (I + [ω]×)
    _dcm_matrix = _dcm_matrix + _dcm_matrix * temp_matrix;
}

void LINS313_AHRS::normalize()
{
    // 计算正交误差
    const float error = _dcm_matrix.a * _dcm_matrix.b;
    
    // Gram-Schmidt正交化
    const Vector3f t0 = _dcm_matrix.a - (_dcm_matrix.b * (0.5f * error));
    const Vector3f t1 = _dcm_matrix.b - (_dcm_matrix.a * (0.5f * error));
    const Vector3f t2 = t0 % t1;
    
    if (!renorm(t0, _dcm_matrix.a) ||
        !renorm(t1, _dcm_matrix.b) ||
        !renorm(t2, _dcm_matrix.c)) {
        // 重正交化失败，重置
        _initialized = false;
        //gcs().send_text(MAV_SEVERITY_INFO, "LINS313 DCM normalize failed");
    }
}

bool LINS313_AHRS::renorm(const Vector3f &a, Vector3f &result)
{
    const float renorm_val = 1.0f / a.length();
    
    if (!(renorm_val < 2.0f && renorm_val > 0.5f)) {
        if (!(renorm_val < 1.0e6f && renorm_val > 1.0e-6f)) {
            return false;
        }
    }
    
    result = a * renorm_val;
    return true;
}
void LINS313_AHRS::drift_correction(float dt)
{
    if (dt <= 0) return;

    // 检测是否静止：加速度计读数接近重力加速度
    float accel_length = _accel_accum.length();
    float accel_diff = fabsf(accel_length - GRAVITY_MSS);
    bool is_static_now = (accel_diff < 2.0f);  // 静止阈值 2 m/s²
    
    // 调试输出
    // static uint32_t last_debug_time = 0;
     uint32_t now_ms = AP_HAL::millis();
    // if (now_ms - last_debug_time > 1000) {
    //     gcs().send_text(MAV_SEVERITY_INFO, 
    //         "DCM: static=%d, diff=%.2f, kp=%.3f, ki=%.4f",
    //         is_static_now, accel_diff, _adaptive_kp, _adaptive_ki);
    //     last_debug_time = now_ms;
    // }
    
    // 自适应增益调整
    if (is_static_now && _was_dynamic) {
        // 刚刚从运动变为静止，大幅增加增益
        _adaptive_kp = _kp * 10.0f;   // 10倍比例增益
        _adaptive_ki = _ki * 20.0f;   // 20倍积分增益
        _last_static_time = AP_HAL::millis();
        
        //gcs().send_text(MAV_SEVERITY_INFO, "DCM: Entering static mode, increasing gains");
    } 
    else if (is_static_now) {
        // 持续静止状态，增益逐渐衰减
        uint32_t static_duration = now_ms - _last_static_time;
        float decay_factor = 1.0f / (1.0f + static_duration / 500.0f);  // 500ms时间常数
        
        _adaptive_kp = _kp * (1.0f + 9.0f * decay_factor);  // 从10倍衰减到1倍
        _adaptive_ki = _ki * (1.0f + 19.0f * decay_factor); // 从20倍衰减到1倍
    } 
    else {
        // 运动状态，使用正常增益
        _adaptive_kp = _kp;
        _adaptive_ki = _ki;
    }
    
    _was_dynamic = !is_static_now;
    
    // 执行偏航漂移校正
    drift_correction_yaw();
    
    // 加速度计数据转换到地球坐标系
    Vector3f accel_ef = _dcm_matrix * _accel_accum;
    
    // 重力参考向量（注意：LINS313的Z轴向上）
    Vector3f gravity_ref(0, 0, GRAVITY_MSS);
    
    // 归一化测量加速度
    Vector3f accel_ef_norm = accel_ef;
    if (accel_ef_norm.length() > 0.5f) {
        accel_ef_norm.normalize();
        
        // 计算误差向量（叉乘）
        Vector3f error_ef = accel_ef_norm % gravity_ref;
        
        // 限制误差大小
        float error_mag = error_ef.length();
        if (error_mag > 0.2f) {
            error_ef *= 0.2f / error_mag;
        }
        
        // 转换误差到机体坐标系
        Vector3f error_body = _dcm_matrix.mul_transpose(error_ef);
        
        // 更新比例项，使用自适应增益
        float spin_rate = _omega.length();
        _omega_P = error_body * _P_gain(spin_rate) * _adaptive_kp;
        
        // 更新积分项（只在低速旋转时）
        if (spin_rate < radians(SPIN_RATE_LIMIT)) {
            _omega_I += error_body * _adaptive_ki * dt;
            
            // 限制积分项大小，防止发散
            float max_integral = radians(5.0f);  // 最大5°/s的积分项
            if (_omega_I.length() > max_integral) {
                _omega_I.normalize();
                _omega_I *= max_integral;
            }
        }
        
        // 更新误差统计
        _error_rp = 0.9f * _error_rp + 0.1f * error_mag;
    }
    
    // 如果静止时间足够长，可以强制重置积分项
    if (is_static_now && (now_ms - _last_static_time) > 2000) {
        // 静止超过2秒，如果误差仍然很大，重置积分项
        if (_error_rp > 0.1f) {
            _omega_I.zero();
            //gcs().send_text(MAV_SEVERITY_WARNING, "DCM: Reset integral due to large error in static");
        }
    }
}
void LINS313_AHRS::drift_correction_yaw()
{
    // LINS313没有提供绝对偏航参考，所以偏航自由积分
    // 只进行轻微的阻尼以防止发散
    float spin_rate = _omega.length();
    
    // 偏航比例项衰减
    _omega_yaw_P *= 0.97f;
    
    // 偏航积分项只在低速时更新
    if (spin_rate < radians(SPIN_RATE_LIMIT)) {
        // 轻微阻尼防止发散
        _omega_I.z *= 0.999f;
    }
    
    _error_yaw = 0.9f * _error_yaw + 0.1f * fabsf(_omega.z);
}

float LINS313_AHRS::_P_gain(float spin_rate)
{
    if (spin_rate < radians(50.0f)) {
        return 1.0f;
    }
    if (spin_rate > radians(500.0f)) {
        return 10.0f;
    }
    return spin_rate / radians(50.0f);
}

float LINS313_AHRS::_yaw_gain() const
{
    // 简化版本，总是返回固定增益
    return 0.2f;
}

void LINS313_AHRS::check_matrix()
{
    if (_dcm_matrix.is_nan()) {
        _initialized = false;
        return;
    }
    
    // 检查DCM矩阵的c.x值是否在有效范围内
    if (!(_dcm_matrix.c.x < 1.0f && _dcm_matrix.c.x > -1.0f)) {
        normalize();
        
        if (_dcm_matrix.is_nan() || fabsf(_dcm_matrix.c.x) > 10.0f) {
            _initialized = false;
        }
    }
}

void LINS313_AHRS::get_euler_angles(float &roll_rad, float &pitch_rad, float &yaw_rad) const
{
    roll_rad = _roll_rad;
    pitch_rad = _pitch_rad;
    yaw_rad = _yaw_rad;
}

void LINS313_AHRS::get_comparison_data(ComparisonData &data) const
{
    data.time_us = AP_HAL::micros64();
    data.lins_roll_deg = _lins_roll_deg;
    data.lins_pitch_deg = _lins_pitch_deg;
    data.lins_yaw_deg = _lins_yaw_deg;
    
    //这里自己写的DCM预测无用
    data.dcm_roll_deg = degrees(_roll_rad);
    data.dcm_pitch_deg = degrees(_pitch_rad);
    data.dcm_yaw_deg = degrees(_yaw_rad);
    
    // 获取APM飞控的DCM姿态 - 使用公共getter函数
    data.apm_roll_deg = degrees(roll_from_apm_dcm_no_gps);
    data.apm_pitch_deg = degrees(pitch_from_apm_dcm_no_gps);
    data.apm_yaw_deg = degrees(yaw_from_apm_dcm_no_gps);
    
    data.error_rp = _error_rp;
    data.error_yaw = _error_yaw;
}

void LINS313_AHRS::log_comparison(uint64_t time_us)
{
#if HAL_LOGGING_ENABLED
    
    ComparisonData data;
    get_comparison_data(data);

    struct log_LDC pkt = {
        LOG_PACKET_HEADER_INIT(LOG_LDC_MSG), // 初始化包头
        time_us       : AP_HAL::micros64(),         // 当前时间戳
        lins_roll     : data.lins_roll_deg,          // 横滚角
        lins_pitch    : data.lins_pitch_deg,
        lins_yaw      : data.lins_yaw_deg,

        dcm_roll      : data.dcm_roll_deg,
        dcm_pitch     : data.dcm_pitch_deg,
        dcm_yaw       : data.dcm_yaw_deg,

        apm_roll      : data.apm_roll_deg,
        apm_pitch     : data.apm_pitch_deg,
        apm_yaw       : data.apm_yaw_deg,

        error_rp      : data.error_rp,
        error_yaw     : data.error_yaw,
        };
     AP::logger().WriteBlock(&pkt, sizeof(pkt));



#endif
}