#include "ArmHardwareInterface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <chrono>
#include <thread>

static const int ACTIVE_MOTORS[] = {1, 2, 3, 4, 5, 6, 7};
static const bool INVERTED[7] = {true, true, true,
     false, true, false, false};




hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info){
    (void)info;
    arm_controller_ = std::make_unique<ArmController>("/dev/ttyUSB0");
    hw_states_position_.resize(7, 0.0);
    hw_states_velocity_.resize(7, 0.0);
    hw_commands_.resize(7, 0.0);
    hw_commands_velocity_.resize(7, 0.0);
    position_offsets_.resize(7, 0.0);


    node_ = std::make_shared<rclcpp::Node>("arm_hardware_interface");
    mit_enable_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "mit_enable", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            int id = msg->data;
            if (id == 0)             { for (int i = 1; i <= 7; i++) arm_controller_->mitEnable(i); }
            else if (id >= 1 && id <= 7) arm_controller_->mitEnable(id);
        });

    mit_disable_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "mit_disable", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            int id = msg->data;
            if (id == 0)             { for (int i = 1; i <= 7; i++) arm_controller_->mitDisable(i); }
            else if (id >= 1 && id <= 7) arm_controller_->mitDisable(id);
        });

    mit_zero_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
        "mit_zero", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            int id = msg->data;
            if (id == 0) {
                for (int i = 1; i <= 7; i++) arm_controller_->mitZero(i);
                cmd_seeded_.fill(false);
            } else if (id >= 1 && id <= 7) {
                arm_controller_->mitZero(id);
                cmd_seeded_[id - 1] = false;
            }
        });

    arm_send_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "arm_send_enable", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data && !send_enabled_) {
                // Poll each motor for current state before enabling send
                for (int motor_id : ACTIVE_MOTORS)
                    arm_controller_->requestStateAndReceive(motor_id);

                // Seed commands from real feedback positions
                const auto states = arm_controller_->getMITStates();
                for (int motor_id : ACTIVE_MOTORS) {
                    int i = motor_id - 1;
                    if (states[i].valid) {
                        hw_commands_[i]          = hw_states_position_[i];
                        RCLCPP_INFO(node_->get_logger(), "Seed motor %d: cmd=%.3f state=%.3f",
                        motor_id, hw_commands_[i], hw_states_position_[i]);
                        hw_commands_velocity_[i] = 0.0;
                        cmd_seeded_[i]           = true;
                    }
                }
            }
            if (!msg->data) {
                cmd_seeded_.fill(false);  // reset przy wyłączeniu
            }
            send_enabled_ = msg->data;
        });

    executor_.add_node(node_);
    executor_thread_ = std::thread([this]() { executor_.spin(); });

    return hardware_interface::CallbackReturn::SUCCESS;
}

ArmHardwareInterface::~ArmHardwareInterface() {
    executor_.cancel();
    if (executor_thread_.joinable())
        executor_thread_.join();
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
    for (int i = 1; i<= 7; i++) {
        arm_controller_->requestStateAndReceive(i);
        usleep(100000);
    }
    const auto states = arm_controller_->getMITStates();
    for (int i = 0; i < 7; i++) {
      RCLCPP_INFO(node_->get_logger(), "Motor %d: valid=%d pos=%.3f", 
                   i+1, states[i].valid, states[i].position);
    }   

    for (int i = 0; i < 7; i++) {
        if (states[i].valid)
            position_offsets_[i] = states[i].position;
    }
    for (int i = 1; i<= 7; i++) {
        arm_controller_->requestStateAndReceive(i);
        usleep(100000);
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
    for (int i = 1; i <= 7; i++)
        arm_controller_->mitDisable(i);
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (int i = 0; i < 7; i++) {
        state_interfaces.emplace_back("joint" + std::to_string(i + 1), "position", &hw_states_position_[i]);
        state_interfaces.emplace_back("joint" + std::to_string(i + 1), "velocity", &hw_states_velocity_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (int i = 0; i < 7; i++) {
        command_interfaces.emplace_back("joint" + std::to_string(i + 1), "position", &hw_commands_[i]);
        command_interfaces.emplace_back("joint" + std::to_string(i + 1), "velocity", &hw_commands_velocity_[i]);
    }
    return command_interfaces;
}

hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
    const auto states = arm_controller_->getMITStates();
    for (int i = 0; i < 7; i++) {
        if (!states[i].valid) continue;
        double new_pos = states[i].position;
        double new_vel = states[i].velocity;
        if (std::abs(new_pos) <= 12.5)
            hw_states_position_[i] = (new_pos - position_offsets_[i]) * (INVERTED[i] ? -1.0 : 1.0);
        if (std::abs(new_vel) <= 50.0)
            hw_states_velocity_[i] = new_vel;
    }
    return hardware_interface::return_type::OK;
}


hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&) {
    if (!send_enabled_) {
        return hardware_interface::return_type::OK;
    }
    for (int motor_id : ACTIVE_MOTORS) {
        int i = motor_id - 1;
        if (!cmd_seeded_[i]) continue;
        const auto& mp = MOTOR_PARAMS[i];
        float cmd_pos = static_cast<float>(hw_commands_[i]) * (INVERTED[i] ? -1.0f : 1.0f) 
                + static_cast<float>(position_offsets_[i]);
        arm_controller_->sendMITAndReceive(motor_id, cmd_pos,
                                          static_cast<float>(hw_commands_velocity_[i]), mp.kp_cmd, mp.kd_cmd, 0.0f);
    }
    return hardware_interface::return_type::OK;
}

PLUGINLIB_EXPORT_CLASS(ArmHardwareInterface, hardware_interface::SystemInterface)


