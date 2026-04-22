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

bool ArmController::ServoReceiveData(ServoState& state) {
    uint32_t id;
    std::vector<uint8_t> data;
    bool result = can.receive(id, data);
    if(result) {
        state.id = id;
        state.position = 0.0f;
        state.torque = 0.0f;
        state.velocity = 0.0f;
    }
    return result;
}
