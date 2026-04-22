#include <rclcpp/rclcpp.hpp>
#include "ArmController.hpp"
class ArmNode : public rclcpp::Node {
    private:
        ArmController arm_controller;



    public:

        ArmNode() : Node("ArmNode"), arm_controller("can0") {
            RCLCPP_INFO(this->get_logger(), "Arm Node Wystartowal");
        };

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}