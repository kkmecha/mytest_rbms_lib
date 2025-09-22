#include "mbed.h"
#include "PID.hpp"

PID::PID(float _u_max, float _u_min) 
    : u_max(_u_max), u_min(_u_min){
    error = 0;
    integral = 0;
    deriv = 0;
    output = 0;
    pre_error = 0;
    pre_prop = 0;
    prop = 0;
};

float PID::pos_type_pid(float KP, float KI, float KD, float set_point, float actual, float dt){
    error = set_point - actual;
    integral += error * dt;
    deriv = (error - pre_error) / dt;
    output = KP * error + KI * integral + KD * deriv;
    output = clamp(output, u_min, u_max);

    pre_error = error;
    return output;
}

float PID::vel_type_pid(float KP, float KI, float KD, float set_point, float actual, float dt){
    error = set_point - actual;
    prop = error - pre_error;
    deriv = prop - pre_prop;
    float du = KP * prop + KI * error * dt + KD * deriv;
    output += du;
    output = clamp(output, u_min, u_max);

    pre_error = error;
    pre_prop = prop;
    return output;
}

float PID::pi_d(float KP, float KI, float KD, float set_point, float actual, float dt){
    error = set_point - actual;
    integral += error * dt;
    deriv = (actual - pre_actual) / dt;
    output = KP * error + KI * integral - KD * deriv;
    output = clamp(output, u_min, u_max);

    pre_actual = actual;
    return output;
}

float PID::i_pd(float KP, float KI, float KD, float set_point, float actual, float dt){
    error = set_point - actual;
    integral += error * dt;
    deriv = (actual - pre_actual) / dt;
    output = KI * integral - KP * actual - KD * deriv;
    output = clamp(output, u_min, u_max);
    
    return output;
}

float PID::clamp(float value, float min_value, float max_value){
    return max(min(value, max_value), min_value);
}