// このファイルは rbms.h の末尾でインクルードされることを想定

template <int MOTOR_NUM>
rbms<MOTOR_NUM>::rbms(const MotorPreset presets[MOTOR_NUM]) : 
    _can(nullptr), _queue(32 * EVENTS_EVENT_SIZE) {
    
    _pids.reserve(MOTOR_NUM); 
    for (int i = 0; i < MOTOR_NUM; ++i) {
        _presets[i] = presets[i];
        _pids.emplace_back(presets[i].max_torque, presets[i].min_torque);
    }
    
    for (int i = 0; i < MOTOR_NUM; ++i) {
        _rotations[i] = 0;
        _mode[i] = TORQUE;
        _target_value[i] = 0;
        _actual_position[i] = 0;
        _actual_speed[i] = 0;
        _is_trapezoid_running[i] = false;
        _trap_v_max[i] = 360.0f;
        _trap_a[i] = 720.0f;
    }
    _timer.start();
    _dt = 0.0f;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::start(CAN& can) {
    _can = &can;
    _event_thread.start(callback(&_queue, &events::EventQueue::dispatch_forever));
    _can->attach(callback(this, &rbms<MOTOR_NUM>::rx_isr), CAN::RxIrq);
    _control_thread.start(callback(this, &rbms<MOTOR_NUM>::unified_control_task));
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::set_control_mode(int id, Ctrl_Mode mode) {
    if (id < 0 || id >= MOTOR_NUM) return;
    _mode[id] = mode;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::set_target(int id, float value) {
    if (id < 0 || id >= MOTOR_NUM) return;
    _target_value[id] = value;
}

template <int MOTOR_NUM>
float rbms<MOTOR_NUM>::get_position(int id) {
    if (id < 0 || id >= MOTOR_NUM) return 0.0f;
    return _actual_position[id] / (float)get_gear_ratio(id);
}

template <int MOTOR_NUM>
float rbms<MOTOR_NUM>::get_speed(int id) {
    if (id < 0 || id >= MOTOR_NUM) return 0.0f;
    return _actual_speed[id] / (float)get_gear_ratio(id);
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::init_trapezoid_control(int id, float target_position, float v_max, float a) {
    if (id < 0 || id >= MOTOR_NUM) return;

    const float gear_ratio = (float)get_gear_ratio(id);

    _trap_target_pos[id] = target_position * gear_ratio;
    _trap_v_max[id] = v_max * gear_ratio;
    _trap_a[id] = a * gear_ratio;

    _trap_start_pos[id] = _actual_position[id];

    _trap_dist_total[id] = abs(_trap_target_pos[id] - _trap_start_pos[id]);
    _trap_dist_accel[id] = (_trap_v_max[id] * _trap_v_max[id]) / (2.0f * _trap_a[id]);
    _trap_dist_decel[id] = _trap_dist_accel[id];

    if (_trap_dist_total[id] < _trap_dist_accel[id] + _trap_dist_decel[id]) {
        _trap_v_max[id] = sqrt(_trap_a[id] * _trap_dist_total[id]);
        _trap_dist_accel[id] = _trap_dist_total[id] / 2.0f;
        _trap_dist_decel[id] = _trap_dist_total[id] / 2.0f;
    }
    
    _is_trapezoid_running[id] = true;
    _mode[id] = TRAPEZOID;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::rx_isr() {
    _queue.call(callback(this, &rbms<MOTOR_NUM>::decode_rbms_data_handler));
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::decode_rbms_data_handler() {
    CANMessage msg;
    if (_can->read(msg)) {
        if (msg.id < 0x201 || msg.id > (0x200 + MOTOR_NUM)) return;
        const int motor_id = msg.id - 0x201;
        _raw_angle[motor_id] = (msg.data[0] << 8) | msg.data[1];
        _raw_speed[motor_id] = (msg.data[2] << 8) | msg.data[3];
        _raw_torque[motor_id] = (msg.data[4] << 8) | msg.data[5];
        _temperature[motor_id] = msg.data[6];
        int64_t& current_rotation = _rotations[motor_id];
        int16_t angle_diff = _raw_angle[motor_id] - (uint16_t)(current_rotation & 0x1FFF);
        if (angle_diff > 4096) current_rotation -= 8192;
        else if (angle_diff < -4096) current_rotation += 8192;
        current_rotation = (current_rotation & ~0x1FFF) | _raw_angle[motor_id];
        _actual_position[motor_id] = (float)current_rotation * 360.0f / 8192.0f;
        _actual_speed[motor_id] = (float)_raw_speed[motor_id];
    }
}

template <int MOTOR_NUM>
int rbms<MOTOR_NUM>::get_gear_ratio(int id) const {
    return _presets[id].motor_is_m3508 ? 19 : 36;
}

template <int MOTOR_NUM>
int rbms<MOTOR_NUM>::get_max_torque_can_value(int id) const {
    return _presets[id].motor_is_m3508 ? 16384 : 10000;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::encode_rbms_data(const int* motor_torque, CANMessage& msg1, CANMessage& msg2) {
    msg1.id = 0x200; msg1.len = 8;
    msg2.id = 0x1ff; msg2.len = 8;
    memset(msg1.data, 0, sizeof(msg1.data));
    memset(msg2.data, 0, sizeof(msg2.data));
    for (int i = 0; i < MOTOR_NUM; ++i) {
        int value = motor_torque[i];
        const int max_value = get_max_torque_can_value(i);
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
bool rbms<MOTOR_NUM>::send_data(CAN& can, const CANMessage& msg1, const CANMessage& msg2) {
    bool success1 = true, success2 = true;
    if (MOTOR_NUM > 0) success1 = can.write(msg1);
    if (MOTOR_NUM > 4) success2 = can.write(msg2);
    return success1 && success2;
}

template <int MOTOR_NUM>
void rbms<MOTOR_NUM>::unified_control_task() {
    int motor_torque[MOTOR_NUM];
    float set_speed[MOTOR_NUM];
    CANMessage msg1, msg2;
    
    while(true) {
        _dt = _timer.elapsed_time().count() / 1000000.0f;
        _timer.reset();
        for(int id = 0; id < MOTOR_NUM; id++) {
            const MotorPreset& preset = _presets[id];
            const float gear_ratio = (float)get_gear_ratio(id);
            
            const float actual_pos_motor_shaft = _actual_position[id];
            const float actual_spd_motor_shaft = _actual_speed[id];

            switch(_mode[id]) {
                case TORQUE:
                    motor_torque[id] = _target_value[id];
                    break;
                case VELOCITY:
                    motor_torque[id] = _pids[id].vel_type_pid(
                        preset.gains.vel_KP, preset.gains.vel_KI, preset.gains.vel_KD,
                        _target_value[id] * gear_ratio, // 変換
                        actual_spd_motor_shaft, _dt);
                    break;
                case POSITION:
                    set_speed[id] = _pids[id].pos_type_pid(
                        preset.gains.pos_KP, preset.gains.pos_KI, preset.gains.pos_KD,
                        _target_value[id] * gear_ratio, // 変換
                        actual_pos_motor_shaft, _dt);
                    
                    motor_torque[id] = _pids[id].vel_type_pid(
                        preset.gains.vel_KP, preset.gains.vel_KI, preset.gains.vel_KD,
                        set_speed[id], actual_spd_motor_shaft, _dt);
                    break;
                case TRAPEZOID: {
                    if (!_is_trapezoid_running[id]) { motor_torque[id] = 0; break; }
                    float dist_moved = abs(actual_pos_motor_shaft - _trap_start_pos[id]);
                    float dist_remaining = _trap_dist_total[id] - dist_moved;
                    float target_vel_motor_shaft; // モーター軸の目標速度
                    int direction = (_trap_target_pos[id] > _trap_start_pos[id]) ? 1 : -1;

                    if (dist_remaining <= 0) {
                        target_vel_motor_shaft = 0;
                        _is_trapezoid_running[id] = false;
                        _mode[id] = POSITION; // 制御を位置制御に引き継ぐと安定しやすい
                        _target_value[id] = _trap_target_pos[id] / gear_ratio; // シャフト軸の最終位置を目標に設定
                    } else if (dist_remaining <= _trap_dist_decel[id]) {
                        target_vel_motor_shaft = sqrt(2.0f * _trap_a[id] * dist_remaining);
                    } else if (dist_moved < _trap_dist_accel[id]) {
                        target_vel_motor_shaft = sqrt(2.0f * _trap_a[id] * dist_moved);
                    } else {
                        target_vel_motor_shaft = _trap_v_max[id];
                    }
                    
                    float target_rpm_motor_shaft = (target_vel_motor_shaft / 360.0f) * 60.0f * direction;
                    
                    motor_torque[id] = _pids[id].vel_type_pid(
                        preset.gains.vel_KP, preset.gains.vel_KI, preset.gains.vel_KD,
                        target_rpm_motor_shaft, actual_spd_motor_shaft, _dt);
                    break;
                }
            }
        }
        encode_rbms_data(motor_torque, msg1, msg2);
        send_data(*_can, msg1, msg2);
        ThisThread::sleep_for(3ms);
    }
}

