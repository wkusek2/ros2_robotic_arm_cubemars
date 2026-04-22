#pragma once

#include <linux/can.h>
#include <string>
#include "CanBridge.hpp"

class ArmController {
private:
    int motor_ids[6];
    CanBridge can;    
public:
    ArmController(const std::string& interface);


};