#ifndef INCLUDED_RBMS_H
#define INCLUDED_RBMS_H

#include "mbed.h"
#include "events/EventQueue.h"
#include "PID.hpp"
#include <cstdint>
#include <vector>

// モーターのモデル定義
typedef enum {
    M2006, // bool -> false
    M3508  // bool -> true
} Motor_Type;

// 制御モードの定義
typedef enum {
    TORQUE,   // トルク制御
    VELOCITY, // 速度制御
    POSITION, // 位置制御
    TRAPEZOID // 台形加減速制御
} Ctrl_Mode;

// PIDゲインをまとめる構造体
typedef struct {
    float pos_KP;
    float pos_KI;
    float pos_KD;
    float vel_KP;
    float vel_KI;
    float vel_KD;
} PID_Gain;

// モーターの全設定をまとめる「仕様書」構造体
struct MotorPreset {
    bool motor_is_m3508; // モーターの型 (true: M3508, false: M2006)
    float max_torque;    // 最大トルク
    float min_torque;    // 最小トルク
    PID_Gain gains;      // このモーター専用のPIDゲイン

    /**
     * @brief MotorPresetを生成するコンストラクタ
     * @param is_m3508 モーターがM3508ならtrue, M2006ならfalse
     * @param g PIDゲイン
     * @note モーターの型に基づいてトルク制限を自動で設定します。
     */
    MotorPreset(bool is_m3508, const PID_Gain& g)
        : motor_is_m3508(is_m3508), gains(g) 
    {
        if (motor_is_m3508) { // M3508
            max_torque = 16384.0f;
            min_torque = -16384.0f;
        } else { // M2006
            max_torque = 10000.0f;
            min_torque = -10000.0f;
        }
    }

    // デフォルトコンストラクタ (配列の初期化などで必要になる場合がある)
    MotorPreset() : motor_is_m3508(false), max_torque(0.0f), min_torque(0.0f), gains{} {}
};


template <int MOTOR_NUM>
class rbms {
public:
    // コンストラクタはMotorPresetの配列を受け取る
    rbms(const MotorPreset presets[MOTOR_NUM]);

    // --- Public API ---
    void start(CAN& can);
    void set_control_mode(int id, Ctrl_Mode mode);
    void set_target(int id, float value);
    void init_trapezoid_control(int id, float target_position, float v_max, float a);
    float get_position(int id);
    float get_speed(int id);
    
private:
    void rx_isr();
    void decode_rbms_data_handler();
    void encode_rbms_data(const int* motor_torque, CANMessage& msg1, CANMessage& msg2);
    bool send_data(CAN& can, const CANMessage& msg1, const CANMessage& msg2);
    int get_gear_ratio(int id) const;
    int get_max_torque_can_value(int id) const; // CANデータとしての最大値
    void unified_control_task();
    
    // --- メンバー変数 ---
    CAN* _can;
    std::vector<PID> _pids; // モーターごとのPIDインスタンス
    Timer _timer;
    float _dt;

    // モーター設定をクラス内で保持
    MotorPreset _presets[MOTOR_NUM];

    // イベントキューとスレッド
    events::EventQueue _queue;
    Thread _event_thread;
    Thread _control_thread;

    // モーター状態変数
    Ctrl_Mode _mode[MOTOR_NUM];
    float _target_value[MOTOR_NUM];
    float _actual_position[MOTOR_NUM];
    float _actual_speed[MOTOR_NUM];
    int64_t _rotations[MOTOR_NUM];
    uint16_t _raw_angle[MOTOR_NUM];
    int16_t _raw_speed[MOTOR_NUM];
    int16_t _raw_torque[MOTOR_NUM];
    uint8_t _temperature[MOTOR_NUM];
    float _trap_v_max[MOTOR_NUM];
    float _trap_a[MOTOR_NUM];
    float _trap_target_pos[MOTOR_NUM];
    float _trap_start_pos[MOTOR_NUM];
    float _trap_dist_total[MOTOR_NUM];
    float _trap_dist_accel[MOTOR_NUM];
    float _trap_dist_decel[MOTOR_NUM];
    bool  _is_trapezoid_running[MOTOR_NUM];
};

#include "rbms.cpp.h"

#endif // INCLUDED_RBMS_H

