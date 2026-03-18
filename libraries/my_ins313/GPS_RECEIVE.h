#pragma once

#include <AP_HAL/AP_HAL.h>

#ifndef GPS_RECEIVE_ENABLED
#define GPS_RECEIVE_ENABLED 1
#endif

class GPS_RECEIVE
{
public:
    GPS_RECEIVE();
    void init();
    void update();

private:
    bool warned = false;
};
