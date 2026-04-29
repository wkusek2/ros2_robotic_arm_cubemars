#pragma once

// ============================================================
// ArmController — sterowanie silnikami ramienia 6-DOF przez CAN
// ============================================================
// Warstwa posrednia miedzy wezlem ROS2 a adapterem USB-CAN.
// Obsluguje dwa tryby komunikacji z silnikami CubeMars AK45-36:
//
//   Tryb serwo (VESC CAN):
//     - CMD_SET_POS    (cmd=4): zadanie pozycji w stopniach
//     - CMD_SET_CURRENT (cmd=1): zadanie pradu w mA
//     - Feedback: extended frame 0x29xx, dane co ~150 Hz
//
//   Tryb MIT (CubeMars MIT Mode):
//     - Ramka 8B: [p(16b) | v(12b) | kp(12b) | kd(12b) | tau(12b)]
//     - Enable/disable/zero przez specjalne bajty 0xFC/0xFD/0xFE
//     - Feedback: standard frame, motor_id w data[0]
//
// Bezpieczenstwo watkowe: can_mutex_ chroni dostep do szyny CAN.
// canLoop (watek) i subscriber ROS2 (spin) pisza jednoczesnie.
// ============================================================

#include <array>
#include <string>
#include <mutex>
#include "CanBridge.hpp"

// --- Struktury danych feedbacku ---

// Stan silnika w trybie serwo (ramka 0x29xx).
struct ServoState {
    int    id;        // CAN ID silnika (1-7)
    float  position;  // pozycja [stopnie], raw/10
    float  velocity;  // predkosc [eRPM/10]
    float  torque;    // prad fazowy [A], raw/100
    int8_t temp;      // temperatura sterownika [°C]
};

// Stan silnika w trybie MIT (standard frame).
struct MITState {
    int     id;           // CAN ID silnika (1-7)
    float   position;     // pozycja [rad], zakres ±12.5
    float   velocity;     // predkosc [rad/s], zakres ±50
    float   torque;       // moment [Nm], zakres ±18
    uint8_t temperature;  // temperatura [°C]
    uint8_t error;        // kod bledu (0 = brak)
};

// Wynik receiveAny() — typ odebranej ramki.
enum class FrameType { NONE, SERVO, MIT };

// --- Klasa kontrolera ---

class ArmController {
public:
    static constexpr int NUM_MOTORS = 6;

    explicit ArmController(const std::string& can_port);

    // --- Odbior ---

    // Czyta jedna ramke z CAN i rozpoznaje jej typ (SERVO lub MIT).
    // Wypelnia odpowiedni struct. Wywolywane wylacznie z canLoop.
    FrameType receiveAny(ServoState& servo, MITState& mit);

    // Zwraca bufor ostatnich znanych stanow serwo (indeks = motor_id - 1).
    const std::array<ServoState, NUM_MOTORS>& getStates() const { return states_; }

    // Dostep do adaptera CAN (np. waitForData w canLoop).
    CanBridge& getCan() { return can; }

    // --- Wysylanie — tryb serwo ---

    // Zadanie pozycji [stopnie] przez VESC CAN (CMD=4).
    bool setPosMotor(int motor_id, float degrees);

    // Zadanie pradu [A] przez VESC CAN (CMD=1).
    bool setCurrentMotor(int motor_id, float current);

    // --- Wysylanie — tryb MIT ---

    // Wlacza tryb MIT na silniku (wysyla 7xFF + 0xFC).
    void mitEnable(int motor_id);

    // Wylacza tryb MIT (wysyla 7xFF + 0xFD).
    void mitDisable(int motor_id);

    // Zeruje enkoder w biezacej pozycji (wysyla 7xFF + 0xFE).
    void mitZero(int motor_id);

    // Wysyla komende sterowania MIT: pozycja [rad], predkosc [rad/s],
    // gain pozycji kp, gain predkosci kd, moment feed-forward [Nm].
    void sendMIT(int motor_id, float p, float v, float kp, float kd, float torque);

private:
    int    motor_ids[NUM_MOTORS];
    CanBridge can;
    std::mutex can_mutex_;                        // mutex dla dostepu do can z wielu watkow
    std::array<ServoState, NUM_MOTORS> states_{}; // bufor ostatnich stanow serwo
};
