#include "ArmController.hpp"

#include <algorithm>
#include <iostream>
#include <vector>

// ============================================================
// Stale protokolu CAN — tryb serwo (VESC CAN)
// ============================================================
// ID ramki = (cmd << 8) | motor_id
// Payload: int32 big-endian

static constexpr int CMD_SET_CURRENT = 1;  // payload: prąd [mA]
static constexpr int CMD_SET_POS     = 4;  // payload: pozycja [stopnie * 1e6]

// Maska ID statusu: ramki 0x2901..0x2907 to feedback serwo.
static constexpr uint32_t STATUS_ID_MASK  = 0xFF00;
static constexpr uint32_t STATUS_ID_VALUE = 0x2900;

// ============================================================
// Stale protokolu MIT — zakresy fizyczne (AK45-36)
// ============================================================
// Wartosci float sa pakowane do uint o podanej liczbie bitow
// liniowo miedzy [MIN, MAX]. Rozdzielczosc:
//   pozycja: 25 rad / 65535 = 0.00038 rad
//   predkosc: 100 rad/s / 4095 = 0.024 rad/s
//   moment:   36 Nm / 4095 = 0.009 Nm

static constexpr float P_MIN  = -12.5f, P_MAX  =  12.5f;  // [rad]
static constexpr float V_MIN  = -50.0f, V_MAX  =  50.0f;  // [rad/s]
static constexpr float KP_MIN =   0.0f, KP_MAX = 500.0f;
static constexpr float KD_MIN =   0.0f, KD_MAX =   5.0f;
static constexpr float T_MIN  = -18.0f, T_MAX  =  18.0f;  // [Nm]

// ============================================================
// Pomocnicze funkcje konwersji MIT
// ============================================================

// Konwertuje float z zakresu [xmin, xmax] do uint o 'bits' bitach.
static uint32_t floatToUint(float x, float xmin, float xmax, int bits) {
    x = std::clamp(x, xmin, xmax);
    return static_cast<uint32_t>((x - xmin) / (xmax - xmin) * ((1 << bits) - 1));
}

// Konwertuje uint z 'bits' bitow z powrotem do float.
static float uintToFloat(uint32_t x, float xmin, float xmax, int bits) {
    return static_cast<float>(x) * (xmax - xmin) / static_cast<float>((1 << bits) - 1) + xmin;
}

// ============================================================
// Inicjalizacja
// ============================================================

ArmController::ArmController(const std::string& can_port) {
    for (int i = 0; i < NUM_MOTORS; i++)
        motor_ids[i] = i + 1;  // silniki 1..6

    can.open(can_port);
}

// ============================================================
// Odbior — receiveAny
// ============================================================
// Czyta dokladnie jedna ramke z CAN i rozpoznaje jej typ:
//   SERVO — extended ID z gornym bajtem 0x29, payload >=7B
//   MIT   — standard ID (<=0x7FF), payload ==8B z motor_id w data[0]
//   NONE  — timeout, blad synchronizacji lub nieznany typ

FrameType ArmController::receiveAny(ServoState& servo, MITState& mit) {
    uint32_t id;
    std::vector<uint8_t> data;

    if (!can.receive(id, data)) return FrameType::NONE;

    // --- Ramka serwo ---
    if ((id & STATUS_ID_MASK) == STATUS_ID_VALUE && data.size() >= 7) {
        servo.id       = static_cast<int>(id & 0xFF);
        servo.position = static_cast<int16_t>((data[0] << 8) | data[1]) / 10.0f;
        servo.velocity = static_cast<int16_t>((data[2] << 8) | data[3]) / 10.0f;
        servo.torque   = static_cast<int16_t>((data[4] << 8) | data[5]) / 100.0f;
        servo.temp     = static_cast<uint8_t>(data[6]);

        int idx = servo.id - 1;
        if (idx >= 0 && idx < NUM_MOTORS)
            states_[idx] = servo;

        return FrameType::SERVO;
    }

    // --- Ramka MIT ---
    // Uklad bitow payload (8 bajtow):
    //   [0..1]       pozycja   16b
    //   [2] + [3>>4] predkosc  12b
    //   [3&F] + [4]  kp        12b  (w feedbacku: unused, ale format taki sam)
    //   [5] + [6>>4] kd        12b
    //   [6&F] + [7]  moment    12b
    if (id <= 0x7FF && data.size() == 8) {
        uint32_t p_i = (static_cast<uint32_t>(data[1]) << 8) | data[2];
        uint32_t v_i = (static_cast<uint32_t>(data[3]) << 4) | (data[4] >> 4);
        uint32_t t_i = ((static_cast<uint32_t>(data[4]) & 0x0F) << 8) | data[5];

        mit.id          = data[0];
        mit.position    = uintToFloat(p_i, P_MIN, P_MAX, 16);
        mit.velocity    = uintToFloat(v_i, V_MIN, V_MAX, 12);
        mit.torque      = uintToFloat(t_i, T_MIN, T_MAX, 12);
        mit.temperature = data[6];
        mit.error       = data[7];

        return FrameType::MIT;
    }

    return FrameType::NONE;
}

// ============================================================
// Wysylanie — tryb serwo
// ============================================================

bool ArmController::setPosMotor(int motor_id, float degrees) {
    int32_t raw = static_cast<int32_t>(degrees * 1000000.0f);
    std::vector<uint8_t> packet = {
        static_cast<uint8_t>((raw >> 24) & 0xFF),
        static_cast<uint8_t>((raw >> 16) & 0xFF),
        static_cast<uint8_t>((raw >>  8) & 0xFF),
        static_cast<uint8_t>((raw >>  0) & 0xFF),
    };
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.send((CMD_SET_POS << 8) | motor_id, packet);
    return true;
}

bool ArmController::setCurrentMotor(int motor_id, float current) {
    int32_t ma = static_cast<int32_t>(current);
    std::vector<uint8_t> packet = {
        static_cast<uint8_t>((ma >> 24) & 0xFF),
        static_cast<uint8_t>((ma >> 16) & 0xFF),
        static_cast<uint8_t>((ma >>  8) & 0xFF),
        static_cast<uint8_t>((ma >>  0) & 0xFF),
    };
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.send((CMD_SET_CURRENT << 8) | motor_id, packet);
    return true;
}

// ============================================================
// Wysylanie — tryb MIT
// ============================================================
// Komendy specjalne: 7xFF + ostatni bajt rozroznia akcje.
// Ramka komendy MIT: 8 bajtow z upakowanymi polami float->uint.

static const std::vector<uint8_t> MIT_ENABLE  = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
static const std::vector<uint8_t> MIT_DISABLE = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
static const std::vector<uint8_t> MIT_ZERO    = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE};

void ArmController::mitEnable(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_ENABLE);
}

void ArmController::mitDisable(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_DISABLE);
}

void ArmController::mitZero(int motor_id) {
    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, MIT_ZERO);
}

// Pakowanie komendy MIT w 8 bajtow:
//   data[0..1]  pozycja   16b BE
//   data[2]     predkosc  [11:4]
//   data[3]     predkosc  [3:0] | kp [11:8]
//   data[4]     kp        [7:0]
//   data[5]     kd        [11:4]
//   data[6]     kd        [3:0] | moment [11:8]
//   data[7]     moment    [7:0]
void ArmController::sendMIT(int motor_id, float p, float v, float kp, float kd, float torque) {
    uint32_t p_i  = floatToUint(p,      P_MIN,  P_MAX,  16);
    uint32_t v_i  = floatToUint(v,      V_MIN,  V_MAX,  12);
    uint32_t kp_i = floatToUint(kp,     KP_MIN, KP_MAX, 12);
    uint32_t kd_i = floatToUint(kd,     KD_MIN, KD_MAX, 12);
    uint32_t t_i  = floatToUint(torque, T_MIN,  T_MAX,  12);

    std::vector<uint8_t> data(8);
    data[0] =  (p_i >> 8) & 0xFF;
    data[1] =   p_i       & 0xFF;
    data[2] =  (v_i >> 4) & 0xFF;
    data[3] = ((v_i  & 0x0F) << 4) | ((kp_i >> 8) & 0x0F);
    data[4] =  kp_i & 0xFF;
    data[5] =  (kd_i >> 4) & 0xFF;
    data[6] = ((kd_i & 0x0F) << 4) | ((t_i  >> 8) & 0x0F);
    data[7] =   t_i & 0xFF;

    std::lock_guard<std::mutex> lock(can_mutex_);
    can.sendStd(motor_id, data);
}
