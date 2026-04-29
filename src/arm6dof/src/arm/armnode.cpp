#include <rclcpp/rclcpp.hpp>
#include <thread>
#include "ArmController.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "arm6dof/msg/mit_feedback_array.hpp"

// ============================================================
// Wezel ROS2 — ArmNode
// ============================================================
// Watki:
//   spin (glowny) — obsluга subscriberow i timera trajektorii
//   canLoop       — blokujacy odczyt ramek CAN przez poll()
//
// Topici subskrybowane:
//   arm/mit_ctrl    [Float64MultiArray: motor_id, cmd]
//                    cmd: 1=enable, 0=disable, 2=zero
//   arm/mit_hw_cmd  [Float64MultiArray: motor_id, p, v, kp, kd, tau]
//
// Topici publikowane:
//   joint_states        [JointState]         — feedback serwo
//   arm/diagnostics     [DiagnosticArray]    — temperatura serwo
//   arm/mit_feedback    [MITFeedbackArray]   — feedback MIT wszystkich silnikow

class ArmNode : public rclcpp::Node {
public:
    ArmNode() : Node("ArmNode"), arm_controller("/dev/ttyUSB0") {
        RCLCPP_INFO(get_logger(), "ArmNode wystartowal");

        // --- Publishery ---
        joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        diag_pub_  = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("arm/diagnostics", 10);
        mit_fb_buf_.motors.resize(7);
        mit_pub_   = create_publisher<arm6dof::msg::MITFeedbackArray>("arm/mit_feedback", 10);

        // --- Subskrybenty MIT ---

        // arm/mit_ctrl: wlaczanie/wylaczanie/zerowanie silnika
        mit_ctrl_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "arm/mit_ctrl", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 2) return;
                int motor_id = static_cast<int>(msg->data[0]);
                int cmd      = static_cast<int>(msg->data[1]);
                if      (cmd == 1) arm_controller.mitEnable(motor_id);
                else if (cmd == 0) arm_controller.mitDisable(motor_id);
                else if (cmd == 2) arm_controller.mitZero(motor_id);
            });

        // arm/mit_hw_cmd: bezposrednia komenda MIT [motor_id, p, v, kp, kd, tau]
        mit_hw_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "arm/mit_hw_cmd", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() < 6) return;
                int motor_id = static_cast<int>(msg->data[0]);
                arm_controller.sendMIT(motor_id,
                    msg->data[1], msg->data[2],
                    msg->data[3], msg->data[4], msg->data[5]);
            });

        // --- Watek CAN ---
        can_thread_ = std::thread(&ArmNode::canLoop, this);
    }

    ~ArmNode() {
        if (can_thread_.joinable()) can_thread_.join();
    }

private:
    ArmController arm_controller;
    std::thread   can_thread_;

    // Publishery
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr        joint_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Publisher<arm6dof::msg::MITFeedbackArray>::SharedPtr      mit_pub_;
    arm6dof::msg::MITFeedbackArray mit_fb_buf_;  // bufor aktualizowany per silnik

    // Subskrybenty
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mit_ctrl_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr mit_hw_sub_;

    // --------------------------------------------------------
    // canLoop — watek odczytu ramek CAN
    // --------------------------------------------------------
    // Blokuje na poll() az pojawia sie dane (oszczedza CPU).
    // Rozpoznaje typ ramki przez receiveAny():
    //   SERVO — publikuje JointState i DiagnosticArray
    //   MIT   — aktualizuje bufor mit_fb_buf_ i publikuje MITFeedbackArray
    void canLoop() {
        while (rclcpp::ok()) {
            if (!arm_controller.getCan().waitForData(200))
                continue;

            ServoState servo;
            MITState   mit;
            switch (arm_controller.receiveAny(servo, mit)) {

            // --- Feedback serwo ---
            case FrameType::SERVO: {
                sensor_msgs::msg::JointState msg;
                msg.header.stamp = now();
                msg.name     = { "joint_" + std::to_string(servo.id) };
                msg.position = { servo.position };
                msg.velocity = { servo.velocity };
                msg.effort   = { servo.torque };
                joint_pub_->publish(msg);

                diagnostic_msgs::msg::DiagnosticArray diag_msg;
                diag_msg.header.stamp = now();
                diagnostic_msgs::msg::DiagnosticStatus status;
                status.name  = "servo_" + std::to_string(servo.id);
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                diagnostic_msgs::msg::KeyValue kv;
                kv.key   = "temp_C";
                kv.value = std::to_string(servo.temp);
                status.values.push_back(kv);
                diag_msg.status.push_back(status);
                diag_pub_->publish(diag_msg);
                break;
            }

            // --- Feedback MIT ---
            // Aktualizuje slot silnika w buforze (indeks = motor_id - 1)
            // i publikuje caly bufor — kazdy silnik ma swoj motors[i].
            case FrameType::MIT: {
                int idx = mit.id - 1;
                if (idx >= 0 && idx < 7) {
                    auto& m       = mit_fb_buf_.motors[idx];
                    m.motor_id    = mit.id;
                    m.position    = mit.position;
                    m.velocity    = mit.velocity;
                    m.torque      = mit.torque;
                    m.temperature = mit.temperature;
                    m.error       = mit.error;
                }
                mit_pub_->publish(mit_fb_buf_);
                break;
            }

            default: break;
            }
        }
    }
};

// ============================================================
// main
// ============================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmNode>());
    rclcpp::shutdown();
    return 0;
}
