#pragma once

#include <string>
#include <vector>
#include <cstdint>

class CanBridge {
private:
    int serial_fd;

public:
    CanBridge();
    bool open(const std::string& interface);
    void send(uint8_t id ,const std::vector<uint8_t>& data);
    bool receive(uint32_t& id, std::vector<uint8_t>& data);
    void close();
};
