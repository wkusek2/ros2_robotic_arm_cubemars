#pragma once

#include "hardware_interface/system_interface.hpp"
#include "ArmController.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <atomic>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

class ArmHardwareInterface : public hardware_interface::SystemInterface {
    public:
    ~ArmHardwareInterface();
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    private:
    std::vector<double> hw_states_velocity_;
    std::vector<double> hw_states_position_;
    std::vector<double> hw_commands_;
    std::vector<double> hw_commands_velocity_;
    std::vector<double> position_offsets_;

    std::unique_ptr<ArmController> arm_controller_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mit_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mit_disable_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mit_zero_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  arm_send_sub_;
    std::atomic<bool> send_enabled_{false};
    std::array<bool, 7> cmd_seeded_{};
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread executor_thread_;
};