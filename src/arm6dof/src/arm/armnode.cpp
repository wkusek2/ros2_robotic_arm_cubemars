#include <rclcpp/rclcpp.hpp>

class ArmNode : public rclcpp::Node {
    private:



    public:

        ArmNode() : Node("ArmNode") {
            RCLCPP_INFO(this->get_logger(), "Arm Node Wystartowal");
        };

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}