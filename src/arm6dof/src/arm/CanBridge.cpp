#include "CanBridge.hpp"
#include <sys/socket.h>
#include <iostream>
#include <net/if.h>
#include <unistd.h>

CanBridge::CanBridge() {
    socket_fd = -1;

};

bool CanBridge::open(const std::string& interface) {
        
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if(socket_fd == -1) {return false;}
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = if_nametoindex(interface.c_str());
    int result = bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr));
    if(result == -1) {return false;}
    return true;
}

void CanBridge::send(const can_frame& frame) {
    int result = write(socket_fd, &frame, sizeof(frame));
    if(result == -1){ std::cerr << "Blad wysylania CAN\n";}

}

bool CanBridge::receive(can_frame& frame) {
    int result = read(socket_fd, &frame, sizeof(frame));
    if(result == -1){ std::cerr << "Blad odbierania CAN\n";}
    return result > 0;
}

void CanBridge::close() {
    ::close(socket_fd);
    socket_fd = -1;
}
