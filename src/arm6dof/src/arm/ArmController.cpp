#include "ArmController.hpp"
#include <string>

ArmController::ArmController(const std::string& interface) {
    motor_ids[0] = 0x01;
    motor_ids[1] = 0x02;
    motor_ids[2] = 0x03;
    motor_ids[3] = 0x04;
    motor_ids[4] = 0x05;
    motor_ids[5] = 0x06;

    can.open(interface);

};