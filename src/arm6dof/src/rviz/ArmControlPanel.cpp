#include "ArmControlPanel.hpp"

#include <rviz_common/display_context.hpp>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QFrame>
#include <pluginlib/class_list_macros.hpp>

ArmControlPanel::ArmControlPanel(QWidget* parent)
: rviz_common::Panel(parent) {
    auto* root = new QVBoxLayout(this);
    root->setSpacing(6);
    root->setContentsMargins(8, 8, 8, 8);

    // ── Nadawanie pozycji ─────────────────────────────────────────
    btn_send_ = new QPushButton("▶ Start nadawania");
    btn_send_->setCheckable(false);
    btn_send_->setStyleSheet("QPushButton { background: #8b2020; color: white; font-weight: bold; padding: 6px; }");
    root->addWidget(btn_send_);
    connect(btn_send_, &QPushButton::clicked, this, &ArmControlPanel::toggleSend);

    btn_poll_ = new QPushButton("▶ Start pollowania");
    btn_poll_->setCheckable(false);
    btn_poll_->setStyleSheet("QPushButton { background: #8b2020; color: white; font-weight: bold; padding: 6px; }");
    root->addWidget(btn_poll_);
    connect(btn_poll_, &QPushButton::clicked, this, &ArmControlPanel::togglePoll);

    auto* sep0 = new QFrame(); sep0->setFrameShape(QFrame::HLine); sep0->setFrameShadow(QFrame::Sunken);
    root->addWidget(sep0);

    // ── Global controls ──────────────────────────────────────────
    auto* global_label = new QLabel("<b>Wszystkie silniki</b>");
    root->addWidget(global_label);

    auto* global_row = new QHBoxLayout();
    auto* btn_en_all  = new QPushButton("Enable All");
    auto* btn_dis_all = new QPushButton("Disable All");
    auto* btn_zero_all = new QPushButton("Zero All");
    btn_en_all->setStyleSheet("QPushButton { background: #2d6a2d; color: white; }");
    btn_dis_all->setStyleSheet("QPushButton { background: #8b2020; color: white; }");
    btn_zero_all->setStyleSheet("QPushButton { background: #1a4a7a; color: white; }");
    global_row->addWidget(btn_en_all);
    global_row->addWidget(btn_dis_all);
    global_row->addWidget(btn_zero_all);
    root->addLayout(global_row);

    connect(btn_en_all,   &QPushButton::clicked, this, &ArmControlPanel::enableAll);
    connect(btn_dis_all,  &QPushButton::clicked, this, &ArmControlPanel::disableAll);
    connect(btn_zero_all, &QPushButton::clicked, this, &ArmControlPanel::zeroAll);

    // ── Separator ────────────────────────────────────────────────
    auto* line = new QFrame();
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Sunken);
    root->addWidget(line);

    // ── Per-motor controls ────────────────────────────────────────
    auto* motor_label = new QLabel("<b>Per silnik</b>");
    root->addWidget(motor_label);

    auto* grid = new QGridLayout();
    grid->setHorizontalSpacing(4);
    grid->setVerticalSpacing(3);

    static const char* MOTOR_NAMES[] = {
        "M1 AK45-36", "M2 AK60-39", "M3 AK45-36",
        "M4 AK45-36", "M5 AK45-10", "M6 AK45-10",
        "M7 Gripper"
    };

    for (int id = 1; id <= 7; id++) {
        int row = id - 1;
        grid->addWidget(new QLabel(MOTOR_NAMES[row]), row, 0);

        auto* btn_en  = new QPushButton("En");
        auto* btn_dis = new QPushButton("Dis");
        auto* btn_z   = new QPushButton("Zero");
        btn_en->setFixedWidth(36);
        btn_dis->setFixedWidth(36);
        btn_z->setFixedWidth(44);
        btn_en->setStyleSheet("QPushButton { background: #2d6a2d; color: white; }");
        btn_dis->setStyleSheet("QPushButton { background: #8b2020; color: white; }");
        btn_z->setStyleSheet("QPushButton { background: #1a4a7a; color: white; }");

        grid->addWidget(btn_en,  row, 1);
        grid->addWidget(btn_dis, row, 2);
        grid->addWidget(btn_z,   row, 3);

        // Capture id by value
        connect(btn_en,  &QPushButton::clicked, this, [this, id]() { enableMotor(id); });
        connect(btn_dis, &QPushButton::clicked, this, [this, id]() { disableMotor(id); });
        connect(btn_z,   &QPushButton::clicked, this, [this, id]() { zeroMotor(id); });
    }

    root->addLayout(grid);
    root->addStretch();
    setLayout(root);
}

void ArmControlPanel::onInitialize() {
    node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    enable_pub_  = node_->create_publisher<std_msgs::msg::Int32>("mit_enable",  10);
    disable_pub_ = node_->create_publisher<std_msgs::msg::Int32>("mit_disable", 10);
    zero_pub_    = node_->create_publisher<std_msgs::msg::Int32>("mit_zero",    10);
    send_pub_    = node_->create_publisher<std_msgs::msg::Bool>("arm_send_enable", 10);
    poll_pub_    = node_->create_publisher<std_msgs::msg::Bool>("arm_poll_enable", 10);
}

void ArmControlPanel::publish(
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr& pub, int value)
{
    if (!pub) return;
    std_msgs::msg::Int32 msg;
    msg.data = value;
    pub->publish(msg);
}

void ArmControlPanel::toggleSend() {
    send_active_ = !send_active_;
    std_msgs::msg::Bool msg;
    msg.data = send_active_;
    if (send_pub_) send_pub_->publish(msg);

    if (send_active_) {
        btn_send_->setText("⏹ Stop nadawania");
        btn_send_->setStyleSheet("QPushButton { background: #2d6a2d; color: white; font-weight: bold; padding: 6px; }");
    } else {
        btn_send_->setText("▶ Start nadawania");
        btn_send_->setStyleSheet("QPushButton { background: #8b2020; color: white; font-weight: bold; padding: 6px; }");
    }
}

void ArmControlPanel::togglePoll() {
    poll_active_ = !poll_active_;
    std_msgs::msg::Bool msg;
    msg.data = poll_active_;
    if (poll_pub_) poll_pub_->publish(msg);

    if (poll_active_) {
        btn_poll_->setText("⏸ Stop pollowania");
        btn_poll_->setStyleSheet("QPushButton { background: #2d6a2d; color: white; font-weight: bold; padding: 6px; }");
    } else {
        btn_poll_->setText("▶ Start pollowania");
        btn_poll_->setStyleSheet("QPushButton { background: #8b2020; color: white; font-weight: bold; padding: 6px; }");
    }
}

void ArmControlPanel::enableAll()          { publish(enable_pub_,  0); }
void ArmControlPanel::disableAll()         { publish(disable_pub_, 0); }
void ArmControlPanel::zeroAll()            { publish(zero_pub_,    0); }
void ArmControlPanel::enableMotor(int id)  { publish(enable_pub_,  id); }
void ArmControlPanel::disableMotor(int id) { publish(disable_pub_, id); }
void ArmControlPanel::zeroMotor(int id)    { publish(zero_pub_,    id); }

PLUGINLIB_EXPORT_CLASS(ArmControlPanel, rviz_common::Panel)
