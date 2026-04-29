#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "arm6dof/msg/mit_feedback_array.hpp"

class ArmNode : public rclcpp::Node {
public:
    ArmNode() : Node("ArmNode") {
        RCLCPP_INFO(get_logger(), "ArmNode started");

        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        diag_pub_  = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("arm/diagnostics", 10);
        mit_fb_buf_.motors.resize(7);
        mit_pub_   = create_publisher<arm6dof::msg::MITFeedbackArray>("arm/mit_feedback", 10);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<arm6dof::msg::MITFeedbackArray>::SharedPtr      mit_pub_;
    arm6dof::msg::MITFeedbackArray mit_fb_buf_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}
