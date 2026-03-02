#include "LINS313_AHRS.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/definitions.h>

extern const AP_HAL::HAL& hal;

LINS313_AHRS::LINS313_AHRS() :
    _lins_roll_deg(0),
    _lins_pitch_deg(0),
    _lins_yaw_deg(0)
{

    _roll_rad = _pitch_rad = _yaw_rad = 0;

}

void LINS313_AHRS::set_lins_reference(float roll_deg, float pitch_deg, float yaw_deg)
{
    _lins_roll_deg = roll_deg;
    _lins_pitch_deg = pitch_deg;
    _lins_yaw_deg = yaw_deg;


}
void LINS313_AHRS::set_lins_reference_apm_no_gps(float roll_deg_no_gps, float pitch_deg_no_gps, float yaw_deg_no_gps)
{
    _roll_rad = radians(roll_deg_no_gps);
    _pitch_rad = radians(pitch_deg_no_gps);
    _yaw_rad = radians(yaw_deg_no_gps);
}

void LINS313_AHRS::get_comparison_data(ComparisonData &data)
{
    data.time_us = AP_HAL::micros64();
    data.lins_roll_deg = _lins_roll_deg;
    data.lins_pitch_deg = _lins_pitch_deg;
    data.lins_yaw_deg = _lins_yaw_deg;
    
    data.dcm_roll_deg = degrees(_roll_rad);
    data.dcm_pitch_deg = degrees(_pitch_rad);
    data.dcm_yaw_deg = degrees(_yaw_rad);
    
    //在这里我需要获取的是DCM的结果而非AHRS的结果，因为AHRS可能会切换到NavEKF3等其他算法
    data.apm_roll_deg = degrees(AP::ahrs().get_roll());
    data.apm_pitch_deg = degrees(AP::ahrs().get_pitch());
    data.apm_yaw_deg = degrees(AP::ahrs().get_yaw());
    
    data.error_rp = 0;
    data.error_yaw = 0;
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