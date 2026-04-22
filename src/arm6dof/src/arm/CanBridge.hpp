#pragma once

#include <linux/can.h>
#include <string>

class CanBridge {
private:
    int socket_fd;

public:
    CanBridge();
    bool open(const std::string& interface);
    void send(const can_frame& frame);
    bool receive(can_frame& frame);
    void close();
};
