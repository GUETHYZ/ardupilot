#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>

class GPS_SEND
{
public:
    GPS_SEND();

    void init(void);
    void update(void);

private:
    AP_HAL::UARTDriver *uart = nullptr;

    uint32_t last_send_ms = 0;
    uint32_t send_interval_ms = 100;   // 10Hz NAV 输出

    struct UBXParserState {
        uint8_t step = 0;
        uint8_t msg_class = 0;
        uint8_t msg_id = 0;
        uint16_t payload_len = 0;
        uint16_t payload_ofs = 0;
        uint8_t ck_a = 0;
        uint8_t ck_b = 0;
        uint8_t rx_ck_a = 0;
        uint8_t rx_ck_b = 0;
        uint8_t payload[256]{};
    } rx{};

    void process_rx(void);
    void handle_rx_byte(uint8_t c);
    void handle_packet(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t len);

    bool send_ubx(uint8_t msg_class, uint8_t msg_id, const void *payload, uint16_t len);
    void send_ack(uint8_t cls_id, uint8_t msg_id, bool ack);
    void send_cfg_prt(void);
    void send_cfg_rate(void);
    void send_nav_packets(uint32_t now_ms);

    static void checksum_update(uint8_t data, uint8_t &ck_a, uint8_t &ck_b);
};
