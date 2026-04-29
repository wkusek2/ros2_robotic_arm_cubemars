#pragma once

// ============================================================
// CanBridge — warstwa komunikacji z adapterem Waveshare USB-CAN-A
// ============================================================
// Adapter laczy sie przez port szeregowy (USB-CDC) z baudrate 2 Mbps.
// Kazda ramka CAN jest opakowana w ramke adapterowa:
//
//   Wysylanie extended 29-bit (tryb serwo):
//     AA | (E0 | len) | id[4B LE] | data[len] | 55
//
//   Wysylanie standard 11-bit (tryb MIT):
//     AA | (C0 | len) | id_low | id_high | data[len] | 55
//
//   Odbior (auto-detekcja standard/extended po bicie 5 bajtu typu):
//     AA | type_byte | id[2 lub 4 B LE] | data[len] | 55
// ============================================================

#include <cstdint>
#include <string>
#include <vector>

class CanBridge {
public:
    CanBridge();
    ~CanBridge();

    // --- Inicjalizacja ---

    // Otwiera port szeregowy i ustawia baudrate 2 Mbps.
    bool open(const std::string& port);

    // --- Wysylanie ---

    // Ramka extended 29-bit — uzywana w trybie serwo (CMD_SET_POS, CMD_SET_CURRENT).
    void send(uint32_t id, const std::vector<uint8_t>& data);

    // Ramka standard 11-bit — uzywana w trybie MIT (komendy sterowania i enable/disable).
    void sendStd(uint16_t id, const std::vector<uint8_t>& data);

    // --- Odbior ---

    // Blokuje watek az pojawia sie dane na FD (lub minie timeout_ms).
    // Uzyc przed receive() zeby uniknac busy-loop i zbednego zuzycia CPU.
    bool waitForData(int timeout_ms);

    // Czyta jedna kompletna ramke CAN. Automatycznie rozpoznaje standard/extended
    // po bicie 5 bajtu typu. Zwraca false przy timeoucie lub bledzie synchronizacji.
    bool receive(uint32_t& id, std::vector<uint8_t>& data);

private:
    int serial_fd;

    bool readByte(uint8_t& b);
    void close();
};
