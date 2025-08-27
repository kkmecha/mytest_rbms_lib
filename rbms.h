#ifndef INCLUDED_RBMS_H
#define INCLUDED_RBMS_H

#include "mbed.h"
#include <cstdint>

template <int MOTOR_NUM>
class rbms {
public:
    rbms(const bool* motor_type);
    void encode_rbms_data(const int* motor, CANMessage& msg1, CANMessage& msg2);
    bool decode_rbms_data(const CANMessage& msg, int& motor_id, int64_t& rotation, short& speed);
    float vel_pid(float T, short rpm_now, short set_speed, float* delta_rpm_pre, float* ie, float KP = 25.0f, float KI = 10.0f, float KD = 0.0f);

private:
    const bool* _motor_type;
    int64_t _rotations[MOTOR_NUM];
    uint16_t _raw_angle;
    int16_t _raw_speed;
    int16_t _raw_torque;
    uint8_t _temperature;
};

#include "rbms.cpp.h"

#endif // INCLUDED_RBMS_H