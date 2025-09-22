#ifndef __PID_HPP__
#define __PID_HPP__

#include "mbed.h"
#include <cmath>

class PID{
    public:
        PID(float _u_max, float _u_min);
        float pos_type_pid(float kp, float ki, float kd, float set_point, float actual, float dt);      // 位置型PID
        float vel_type_pid(float KP, float KI, float KD, float set_point, float actual, float dt);      // 速度型PID
        float pi_d(float KP, float KI, float KD, float set_point, float actual, float dt);              // 微分先行型PID
        float i_pd(float KP, float KI, float KD, float set_point, float actual, float dt);              // 比例微分先行型PID
    private: 
        float clamp(float value, float min_value, float max_value);
        float error;
        float pre_error;
        float prop;
        float pre_prop;
        float pre_actual;
        float integral;
        float deriv;
        float output;
        float u_min, u_max;
};

#endif // __PID_HPP__