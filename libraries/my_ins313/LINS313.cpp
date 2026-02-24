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
    uart = my_313_p.find_serial(AP_SerialManager::SerialProtocol_MY_LINS313,0);
    if (uart == nullptr) 
    {
        gcs().send_text(MAV_SEVERITY_INFO,"LINS313 Uart Init Error");
        return;
    }

}
void MY_LINS313::update(void)
{
    if (uart == nullptr) 
     {
         // 调试输出：UART为空（每秒一次）
         static uint32_t last_no_uart_time = 0;
         uint32_t now = AP_HAL::millis();
         if (now - last_no_uart_time > 1000) {
             gcs().send_text(MAV_SEVERITY_INFO, "No LINS313 Uart");
             last_no_uart_time = now;
         }
         return;
     }
    
    // 读取串口缓冲区的数据
    uint32_t nbytes = MIN(uart->available(), 2048u);
    while (nbytes-- > 0) 
    {
        int16_t r = uart->read();
        if (r < 0) 
        {
            // 调试输出：读取错误（每秒一次）
            static uint32_t last_read_error_time = 0;
            uint32_t now = AP_HAL::millis();
            if (now - last_read_error_time > 1000) {
                gcs().send_text(MAV_SEVERITY_INFO, "LINS313 Data r<0");
                last_read_error_time = now;
            }
            break;
        }
        uint8_t c = (uint8_t)r;
        if (recv_count == 0) 
        {
            // 帧头字节0
            if(c == LINS313_HEAD0) 
            {
                recv_count++;
                sum = 0;
            }
        } 
        else if (recv_count == 1) 
         {
             // 帧头字节1
             if (c != LINS313_HEAD1) 
             {
                 recv_count = 0;  // 重置状态机
             } 
             else 
             {
                 // 调试输出：收到帧头1（每秒一次）
                //  static uint32_t last_head1_time = 0;
                //  uint32_t now = AP_HAL::millis();
                //  if (now - last_head1_time > 1000) {
                //      gcs().send_text(MAV_SEVERITY_INFO, "LINS313 Data Head1 Recive");
                //      last_head1_time = now;
                //  }
                 recv_count++;
             }
         } 
        else if (recv_count < sizeof(pack_313_update) + 2) 
         {
             // recv_count从2开始，减去2得到结构体中的偏移
             // 接收数据并存入结构体
            
             ((uint8_t*)&pack_313_update)[recv_count - 2] = c;
             sum += c;
             recv_count++;
         } 
        else if (recv_count == sizeof(pack_313_update) + 2) 
        {
             // 校验和检查
            uint8_t calculated_checksum = ~(uint8_t)sum;
            
            if (calculated_checksum == c) 
             {
                 // 校验通过，解析数据
                 data_analyze_roll = 0.0001f * (int32_t)(
                     (pack_313_update.roll[0] << 24) | 
                     (pack_313_update.roll[1] << 16) | 
                     (pack_313_update.roll[2] << 8)  | 
                     (pack_313_update.roll[3]));                   
                 data_analyze_pitch = 0.0001f * (int32_t)(
                     (pack_313_update.pitch[0] << 24) | 
                     (pack_313_update.pitch[1] << 16) | 
                     (pack_313_update.pitch[2] << 8)  | 
                     (pack_313_update.pitch[3]));                   
                 data_analyze_yaw = 0.0001f * (int32_t)(
                     (pack_313_update.yaw[0] << 24) | 
                     (pack_313_update.yaw[1] << 16) | 
                     (pack_313_update.yaw[2] << 8)  | 
                     (pack_313_update.yaw[3]));

                data_analyze_acc_x = 0.0001f * (int32_t)(
                     (pack_313_update.accx[0] << 24) | 
                     (pack_313_update.accx[1] << 16) | 
                     (pack_313_update.accx[2] << 8)  | 
                     (pack_313_update.accx[3]));
                data_analyze_acc_y = 0.0001f * (int32_t)(
                     (pack_313_update.accy[0] << 24) | 
                     (pack_313_update.accy[1] << 16) | 
                     (pack_313_update.accy[2] << 8)  | 
                     (pack_313_update.accy[3]));
                data_analyze_acc_z = 0.0001f * (int32_t)(
                     (pack_313_update.accz[0] << 24) | 
                     (pack_313_update.accz[1] << 16) | 
                     (pack_313_update.accz[2] << 8)  | 
                     (pack_313_update.accz[3]));  

                data_analyze_gyro_x = 0.0001f * (int32_t)(
                     (pack_313_update.gyro_x[0] << 24) | 
                     (pack_313_update.gyro_x[1] << 16) | 
                     (pack_313_update.gyro_x[2] << 8)  | 
                     (pack_313_update.gyro_x[3]));                      
                data_analyze_gyro_y = 0.0001f * (int32_t)(
                     (pack_313_update.gyro_y[0] << 24) | 
                     (pack_313_update.gyro_y[1] << 16) | 
                     (pack_313_update.gyro_y[2] << 8)  | 
                     (pack_313_update.gyro_y[3])); 
                data_analyze_gyro_z = 0.0001f * (int32_t)(
                     (pack_313_update.gyro_z[0] << 24) | 
                     (pack_313_update.gyro_z[1] << 16) | 
                     (pack_313_update.gyro_z[2] << 8)  | 
                     (pack_313_update.gyro_z[3]));                        
                
                phrased = true;

                uint64_t now_us = AP_HAL::micros64();
                float dt = (_last_update_time_us > 0) ? (now_us - _last_update_time_us) * 1e-6f : 0.005f; // 默认5ms (200Hz)
                _last_update_time_us = now_us;
    
                // 准备传感器数据（注意单位转换和坐标系转换）
                // 这里可以快速切换为其他传感器的数据
                // LINS313数据单位：加速度(g)，陀螺仪(°/s)
                // 需要转换为：加速度(m/s²)，陀螺仪(rad/s)
                        
                // Z轴方向转换：LINS313的Z向上，APM的Z向下，所以Z轴加速度和角速度需要取反
                Vector3f accel(data_analyze_acc_x * GRAVITY_MSS, 
                            data_analyze_acc_y * GRAVITY_MSS, 
                            -data_analyze_acc_z * GRAVITY_MSS);  // Z取反
    
                Vector3f gyro(radians(data_analyze_gyro_x),
                            radians(data_analyze_gyro_y),
                            -radians(data_analyze_gyro_z));  // Z取反
    
                // 设置LINS313直接输出作为参考
                _ahrs.set_lins_reference(data_analyze_roll, data_analyze_pitch, data_analyze_yaw);
    
                // 更新DCM解算
                _ahrs.update(gyro, accel, dt);
    
                // 记录对比日志
                _ahrs.log_comparison(now_us);
    
                // 调试输出
                static uint32_t last_debug_time = 0;
                uint32_t now_ms = AP_HAL::millis();
                if (now_ms - last_debug_time > 1000) 
                {
                    if (_ahrs.is_initialized()) 
                    {
                        float dcm_roll, dcm_pitch, dcm_yaw;
                        _ahrs.get_euler_angles(dcm_roll, dcm_pitch, dcm_yaw);
            
                        /*gcs().send_text(MAV_SEVERITY_INFO,
                            "LINS313: L[%.1f,%.1f,%.1f] D[%.1f,%.1f,%.1f]",
                            data_analyze_roll, data_analyze_pitch, data_analyze_yaw,
                        degrees(dcm_roll), degrees(dcm_pitch), degrees(dcm_yaw));*/
                    }
                last_debug_time = now_ms;   
                } 

            else // 校验失败
             {
                 
             }
            
            // 无论校验是否通过，都重置状态机
            recv_count = 0;
             phrased = false;
         }
            else
            {
                // 这里没有输出，无需添加频率限制
            }
        }
    }
}


namespace AP {
    MY_LINS313 *my_lins313()
    {
        return MY_LINS313::get_singleton();
    }
}