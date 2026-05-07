#pragma once

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <QPushButton>

class ArmControlPanel : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit ArmControlPanel(QWidget* parent = nullptr);
    void onInitialize() override;

private Q_SLOTS:
    void enableAll();
    void disableAll();
    void zeroAll();
    void enableMotor(int id);
    void disableMotor(int id);
    void zeroMotor(int id);
    void toggleSend();

private:
    void publish(rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& pub, int value);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr enable_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr disable_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr zero_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  send_pub_;

    QPushButton* btn_send_{nullptr};
    bool send_active_{false};
};
