#ifndef INCLUDED_RBMS_H
#define INCLUDED_RBMS_H

#include "mbed.h"
#include "PID.hpp"
#include <cstdint>

typedef enum {
    M2006,
    M3508
} Motor_Type;

typedef enum {
    TORQUE,
    VELOCITY,
    POSITION,
    TRAPEZOID
} Ctrl_Mode;

typedef struct {
    float pos_KP;
    float pos_KI;
    float pos_KD;
    float vel_KP;
    float vel_KI;
    float vel_KD;
} PID_Gain;

template <int MOTOR_NUM>
class rbms {
public:
    rbms(const bool* motor_type);
    int get_giar_ratio(int id);
    void get_giar_ratio(int *giar_ratio);
    int get_max_torque(int id);
    void get_max_torque(int *max_torque);
    void encode_rbms_data(const int* motor, CANMessage& msg1, CANMessage& msg2);
    bool decode_rbms_data(const CANMessage& msg, int64_t* rotation, short* speed);
    bool send_data(CAN& can, CANMessage msg1, CANMessage msg2);

    void init_trapezoid_control();
    void unified_control(Ctrl_Mode *mode);
private:
    PID _pid;
    PID_Gain _pid_gain;
    const bool* _motor_type;
    int64_t _rotations[MOTOR_NUM];
    uint16_t _raw_angle;
    int16_t _raw_speed;
    int16_t _raw_torque;
    uint8_t _temperature;
};

#include "rbms.cpp.h"

#endif // INCLUDED_RBMS_H