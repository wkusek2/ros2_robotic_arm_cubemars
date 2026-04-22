#include "CanBridge.hpp"
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <cstring>

CanBridge::CanBridge() {
    serial_fd = -1;

};

bool CanBridge::open(const std::string& interface) {
        
    serial_fd = ::open(interface.c_str(), O_RDWR | O_NOCTTY);
    if(serial_fd == -1) { std::cerr<< "Blad otwarcia seriala\n"; return false;}
    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(serial_fd, &tty);
    cfsetospeed(&tty, B2000000);
    cfsetispeed(&tty, B2000000);
    tcsetattr(serial_fd, TCSANOW, &tty);
    return true;
}

void CanBridge::send(uint8_t id, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> packet;
    packet.push_back(0xAA);
    packet.push_back(0xE0 | data.size());
    packet.push_back(id);
    packet.push_back(0x00);
    packet.push_back(0x00);
    packet.push_back(0x00);
    packet.insert(packet.end(), data.begin(),data.end());
    packet.push_back(0x55);
    int result = write(serial_fd, packet.data(), packet.size());
    if(result == -1){ std::cerr << "Blad wysylania CAN\n";}

}

bool CanBridge::receive(uint32_t& id, std::vector<uint8_t>& data) {
    uint8_t header[2];
    if (::read(serial_fd, header, 2) != 2) return false;
    if (header[0] != 0xAA || (header[1] & 0xC0) != 0xC0) return false;

    uint8_t data_len = header[1] & 0x0F;
    std::vector<uint8_t> rest(4 + data_len + 1);
    if (::read(serial_fd, rest.data(), rest.size()) != (int)rest.size()) return false;
    if (rest[4 + data_len] != 0x55) return false;

    id = rest[0] | (rest[1] << 8) | (rest[2] << 16) | (rest[3] << 24);
    data.assign(rest.begin() + 4, rest.begin() + 4 + data_len);
    return true;
}


void CanBridge::close() {
    ::close(serial_fd);
    serial_fd = -1;
}
