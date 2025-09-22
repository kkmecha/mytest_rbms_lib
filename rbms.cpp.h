#include "mbed.h"
#include "PID.hpp"
#include "rbms.h"
#include <cstdint>

template <int MOTOR_NUM>
rbms<MOTOR_NUM>::rbms(const bool* motor_type) : _motor_type(motor_type) {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        _rotations[i] = 0;
    }
}

template <int MOTOR_NUM>
int rbms<MOTOR_NUM>::get_giar_ratio(int id){
    return _motor_type[id] ? 19 : 36;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::get_giar_ratio(int *giar_ratio){
    for(int i = 0; i < MOTOR_NUM; i++) giar_ratio[i] = (_motor_type[i] ? 19 : 36);
}

template <int MOTOR_NUM>
int rbms<MOTOR_NUM>::get_max_torque(int id){
    return _motor_type[id] ? 16384 : 10000;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::get_max_torque(int *max_torque){
    for(int i = 0; i < MOTOR_NUM; i++) max_torque[i] = (_motor_type[i] ? 16384 : 10000);
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::encode_rbms_data(const int* motor, CANMessage& msg1, CANMessage& msg2) {
    msg1.id = 0x200;
    msg1.len = 8;
    msg2.id = 0x1ff;
    msg2.len = 8;
    
    memset(msg1.data, 0, sizeof(msg1.data));
    memset(msg2.data, 0, sizeof(msg2.data));

    for (int i = 0; i < MOTOR_NUM; ++i) {
        int value = motor[i];
        const int max_value = get_max_torque();

        if (value > max_value) value = max_value;
        if (value < -max_value) value = -max_value;
        
        if (i < 4) {
            msg1.data[i * 2]     = (uint8_t)(value >> 8);
            msg1.data[i * 2 + 1] = (uint8_t)(value & 0xFF);
        } else if (i < 8){
            int index = (i - 4) * 2;
            msg2.data[index]     = (uint8_t)(value >> 8);
            msg2.data[index + 1] = (uint8_t)(value & 0xFF);
        }
    }
}

template <int MOTOR_NUM>
bool rbms<MOTOR_NUM>::decode_rbms_data(const CANMessage& msg, int64_t* rotation, short* speed) {
    if (msg.id < 0x201 || msg.id > (0x200 + MOTOR_NUM)) return false;

    motor_id = msg.id - 0x201;

    _raw_angle = (msg.data[0] << 8) | msg.data[1];
    _raw_speed = (msg.data[2] << 8) | msg.data[3];
    _raw_torque = (msg.data[4] << 8) | msg.data[5];
    _temperature = msg.data[6];

    int64_t& current_rotation = _rotations[motor_id]; 
    int64_t high_bits = current_rotation & 0xFFFFFFFFFFFFE000;
    int64_t new_pos_candidate = high_bits | (uint16_t)(_raw_angle & 0x1FFF);
    int64_t diff = new_pos_candidate - current_rotation;

    if (diff > 4096) new_pos_candidate -= 8192;
    else if (diff < -4096) new_pos_candidate += 8192;
    
    current_rotation = new_pos_candidate;
    rotation[motor_id] = (float)current_rotation * 360.0f / 8192.0f;

    speed[motor_id] = _raw_speed; 
    
    return true;
}

template <int MOTOR_NUM>
bool rbms<MOTOR_NUM>::send_data(CAN& can, CANMessage msg1, CANMessage msg2) {
    if(can.write(msg1) || can.write(msg2)) return true;
    else return false;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::init_trapezoid_control() {

}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::unified_control(Ctrl_Mode *mode){
    int motor[MOTOR_NUM], set_speed[MOTOR_NUM];
    int giar_ratio[MOTOR_NUM];
    const int SPEED_MAX = 500; // rpm
    CANMessage msg1, msg2;

    this->get_giar_ratio(giar_ratio);

    while(true){ 
        for(int id = 0; id < MOTOR_NUM; id++){
            if(mode[id] == TORQUE){
                motor[id] = torque[id];
            }else if(mode[id] == VELOCITY){
                motor[id] = _pid.vel_type_pid(_pid_gain.vel_KP, _pid_gain.vel_KI, _pid_gain.vel_KD, set, act/giar_ratio[id], dt);
            }else if(mode[id] == POSITION){
                set_speed[id] = _pid.pos_type_pid(_pid_gain.pos_KP, _pid_gain.pos_KI, _pid_gain.pos_KD, set, act/giar_ratio[id], dt);
                if (set_speed[id] > SPEED_MAX) set_speed[id] = SPEED_MAX;
                else if (set_speed[id] < -SPEED_MAX) set_speed[id] = -SPEED_MAX;
                motor[id] = _pid.vel_type_pid(_pid_gain.vel_KP, _pid_gain.vel_KI, _pid_gain.vel_KD, set_speed[id], act/giar_ratio[id], dt);
            }else if(mode[id] == TRAPEZOID){

            }
        }
        this->encode_rbms_data(motor, msg1, msg2);
        this->send_data(can, msg1, msg2);
        ThisThread::sleep_for(1ms);
    }
};
