// #include "AP_Airspeed_External.h"

// #if AP_AIRSPEED_EXTERNAL_ENABLED

// AP_Airspeed_External::AP_Airspeed_External(AP_Airspeed &_frontend, uint8_t _instance) :
//     AP_Airspeed_Backend(_frontend, _instance)
// {
//     set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL,0,_instance,0));
// }

// // return the current differential_pressure in Pascal
// bool AP_Airspeed_External::get_differential_pressure(float &pressure)
// {
//     WITH_SEMAPHORE(sem);
//     if (press_count == 0) {
//         return false;
//     }
//     pressure = sum_pressure/press_count;
//     press_count = 0;
//     sum_pressure = 0;
//     return true;
// }

// // get last temperature
// bool AP_Airspeed_External::get_temperature(float &temperature)
// {
//     WITH_SEMAPHORE(sem);
//     if (temperature_count == 0) {
//         return false;
//     }
//     temperature = sum_temperature/temperature_count;
//     temperature_count = 0;
//     sum_temperature = 0;
//     return true;
// }

// void AP_Airspeed_External::handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt)
// {
//     WITH_SEMAPHORE(sem);

//     sum_pressure += pkt.differential_pressure;
//     press_count++;
//     if (press_count > 100) {
//         // prevent overflow
//         sum_pressure /= 2;
//         press_count /= 2;
//     }

//     sum_temperature += pkt.temperature;
//     temperature_count++;
//     if (temperature_count > 100) {
//         // prevent overflow
//         sum_temperature /= 2;
//         temperature_count /= 2;
//     }
// }

// #endif // AP_AIRSPEED_EXTERNAL_ENABLED


#include "AP_Airspeed_External.h"

#if AP_AIRSPEED_EXTERNAL_ENABLED

#include <AP_HAL/AP_HAL.h>

AP_Airspeed_External::AP_Airspeed_External(AP_Airspeed &_frontend, uint8_t _instance) :
    AP_Airspeed_Backend(_frontend, _instance)
{
    set_bus_id(AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, 0, _instance, 0));
}

bool AP_Airspeed_External::get_differential_pressure(float &pressure)
{
    WITH_SEMAPHORE(sem);

    const uint32_t now_ms = AP_HAL::millis();
    if (last_update_ms == 0 || (now_ms - last_update_ms) > 300) {
        return false;
    }

    pressure = last_pressure;
    return true;
}

bool AP_Airspeed_External::get_temperature(float &temperature)
{
    WITH_SEMAPHORE(sem);

    const uint32_t now_ms = AP_HAL::millis();
    if (last_update_ms == 0 || (now_ms - last_update_ms) > 300) {
        return false;
    }

    temperature = last_temperature;
    return true;
}

void AP_Airspeed_External::handle_external(const AP_ExternalAHRS::airspeed_data_message_t &pkt)
{
    WITH_SEMAPHORE(sem);

    last_pressure = pkt.differential_pressure;
    last_temperature = pkt.temperature;
    last_update_ms = AP_HAL::millis();
}

#endif