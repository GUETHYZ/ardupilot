#include "LINS313_SEND.h"
#include "LINS313.h"                // 提供 MY_LINS313 单例及 ComparisonData
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

MY_LINS313_SEND::MY_LINS313_SEND() :
    uart(nullptr),
    last_send_ms(0)
{}

void MY_LINS313_SEND::init()
{

    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_LINS313_SEND, 0);

    if (uart == nullptr) {
        // 调试输出：未找到串口
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LINS313_SEND: no UART found");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LINS313_SEND:  UART initialized");
    // // 打开串口，使用默认的波特率（例如 115200）
    // uart->begin(sm.find_baudrate(AP_SerialManager::SerialProtocol_LINS313_SEND, 0));
}

void MY_LINS313_SEND::update()
{
    if (uart == nullptr) {
        return;
    }

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_send_ms < send_interval_ms) {
        return;
    }
    last_send_ms = now_ms;

    // 获取 LINS313 单例
    MY_LINS313 *lins = AP::my_lins313();
    if (lins == nullptr) {
        return;
    }

    // 获取对比数据（需在 MY_LINS313 中增加 get_comparison_data 方法）
    LINS313_AHRS::ComparisonData data;
    if (!lins->get_comparison_data(data)) {
        // 数据无效（例如传感器未解析到有效帧），跳过本次发送
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LINS313_SEND: no valid data");
        return;
    }

    // 构建发送缓冲区
    uint8_t buf[27];  // 2帧头 + 24数据 + 1校验和
    buf[0] = 0x7F;    // 帧头0
    buf[1] = 0x94;    // 帧头1

    // 将6个float转换为大端字节序存入数据区（偏移2开始）
    float_to_be_bytes(data.lins_roll_deg, &buf[2]);
    float_to_be_bytes(data.lins_pitch_deg, &buf[6]);
    float_to_be_bytes(data.lins_yaw_deg, &buf[10]);
    float_to_be_bytes(data.apm_roll_deg, &buf[14]);
    float_to_be_bytes(data.apm_pitch_deg, &buf[18]);
    float_to_be_bytes(data.apm_yaw_deg, &buf[22]);

    // 计算校验和：数据区所有字节累加，取反
    uint8_t sum = 0;
    for (uint8_t i = 2; i < 26; i++) {
        sum += buf[i];
    }
    buf[26] = ~sum;   // 校验和

    // 通过串口发送
    size_t written = uart->write(buf, sizeof(buf));
    if (written != sizeof(buf)) 
    {
        gcs().send_text(MAV_SEVERITY_WARNING, "LINS313 send partial: %u/%u", 
                    (unsigned)written, (unsigned)sizeof(buf));
    } 
    else 
    {
        // 可选：发送成功时的调试信息
        //gcs().send_text(MAV_SEVERITY_DEBUG, "LINS313 sent OK");
    }
    
    // static uint32_t last_head1_time = 0;
    // uint32_t now = AP_HAL::millis();
    // if (now - last_head1_time > 1000) 
    // {
    //     gcs().send_text(MAV_SEVERITY_INFO, "LINS313 checksum: 0x%02X", buf[26]);
    //     last_head1_time = now;
    // }
}

void MY_LINS313_SEND::float_to_be_bytes(float value, uint8_t bytes[4]) const
{
    // 将float的二进制表示转为uint32_t（保持位模式）
    union {
        float f;
        uint32_t i;
    } u;
    u.f = value;

    // 主机通常为小端，转换为大端（网络字节序）
    bytes[0] = (u.i >> 24) & 0xFF;
    bytes[1] = (u.i >> 16) & 0xFF;
    bytes[2] = (u.i >> 8) & 0xFF;
    bytes[3] = u.i & 0xFF;
}
