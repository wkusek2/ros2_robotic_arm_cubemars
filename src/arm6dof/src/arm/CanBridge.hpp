#pragma once

// ============================================================
// CanBridge — serial communication layer for Waveshare USB-CAN-A
// ============================================================
// The adapter connects via USB-CDC serial at 2 Mbps baudrate.
// Each CAN frame is wrapped in an adapter frame:
//
//   Sending extended 29-bit (servo mode):
//     AA | (E0 | len) | id[4B LE] | data[len] | 55
//
//   Sending standard 11-bit (MIT mode):
//     AA | (C0 | len) | id_low | id_high | data[len] | 55
//
//   Receiving (auto-detects standard/extended from bit 5 of type byte):
//     AA | type_byte | id[2 or 4 B LE] | data[len] | 55
// ============================================================

#include <cstdint>
#include <string>
#include <vector>

class CanBridge {
public:
    CanBridge();
    ~CanBridge();

    // --- Initialization ---

    // Opens the serial port and configures 2 Mbps baudrate.
    bool open(const std::string& port);

    // --- Transmit ---

    // Extended 29-bit frame — used in servo mode (CMD_SET_POS, CMD_SET_CURRENT).
    void send(uint32_t id, const std::vector<uint8_t>& data);

    // Standard 11-bit frame — used in MIT mode (control commands and enable/disable).
    void sendStd(uint16_t id, const std::vector<uint8_t>& data);

    // --- Receive ---

    // Blocks until data arrives on the fd (or timeout_ms elapses).
    // Call before receive() to avoid busy-looping and wasting CPU.
    bool waitForData(int timeout_ms);

    // Reads one complete CAN frame. Auto-detects standard/extended
    // from bit 5 of the type byte. Returns false on timeout or sync error.
    bool receive(uint32_t& id, std::vector<uint8_t>& data);

private:
    int serial_fd;

    bool readByte(uint8_t& b);
    void close();
};
