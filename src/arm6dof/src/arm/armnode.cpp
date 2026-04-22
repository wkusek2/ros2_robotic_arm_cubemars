#include <rclcpp/rclcpp.hpp>
#include "ArmController.hpp"

class ArmNode : public rclcpp::Node {
    private:
        ArmController arm_controller;
        rclcpp::TimerBase::SharedPtr timer_;

        void callback(){
            ServoState state;
            bool status = arm_controller.ServoReceiveData(state);
            if(status) {
            RCLCPP_INFO(this->get_logger(), "ID: %d", state.id);
            }
        }
    public:
        ArmNode() : Node("ArmNode"), arm_controller("/dev/ttyUSB2") {
            RCLCPP_INFO(this->get_logger(), "Arm Node Wystartowal");
            timer_ = create_wall_timer(std::chrono::milliseconds(100),[this]{callback();});
        };

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}