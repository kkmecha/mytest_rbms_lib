template <int MOTOR_NUM>

rbms<MOTOR_NUM>::rbms(const bool* motor_type) : _motor_type(motor_type) {
    for (int i = 0; i < MOTOR_NUM; ++i) {
        _rotations[i] = 0;
    }
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
        const int max_value = _motor_type[i] ? 16384 : 10000;

        if (value > max_value) value = max_value;
        if (value < -max_value) value = -max_value;

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
bool rbms<MOTOR_NUM>::decode_rbms_data(const CANMessage& msg, int& motor_id, int64_t& rotation, short& speed) {
    if (msg.id < 0x201 || msg.id > (0x200 + MOTOR_NUM)){
        return false;
    }

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
    rotation = (float)current_rotation * 360.0f / 8192.0f;

    speed = _raw_speed; 
    
    return true;
}

template <int MOTOR_NUM>
float rbms<MOTOR_NUM>::vel_pid(float T, short rpm_now, short set_speed, float* delta_rpm_pre, float* ie, float KP, float KI, float KD) {
    float delta_rpm = (float)set_speed - (float)rpm_now;
    float de = (delta_rpm - *delta_rpm_pre) / T;
    
    *ie = *ie + (delta_rpm + *delta_rpm_pre) * T / 2.0f;
    // Iゲインの上限下限（ワインドアップ対策）
    // if (*ie > 3000) *ie = 3000;
    // if (*ie < -3000) *ie = -3000;

    float out_torque = KP * delta_rpm + KI * (*ie) + KD * de;
    *delta_rpm_pre = delta_rpm;

    return out_torque;
}