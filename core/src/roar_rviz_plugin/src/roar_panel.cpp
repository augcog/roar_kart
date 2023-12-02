#include "roar_rviz_plugin/roar_panel.hpp"

#include <memory>
#include <string>
#include <iostream>
#include <nav2_util/lifecycle_service_client.hpp>

namespace roar_rviz_plugin
{

    ROARPanel::ROARPanel(QWidget *parent)
        : Panel(parent)
    {
        node = rclcpp::Node::make_shared("ROARPanel");

        client =
            node->create_client<roar_msgs::srv::ToggleControlSafetySwitch>("/roar/controller_manager/safety_toggle");
        this->setupUI();
    }

    ROARPanel::~ROARPanel()
    {
    }

    void ROARPanel::setupUI()
    {
        QVBoxLayout *main_layout = new QVBoxLayout;

        safetyButton = new QPushButton;
        safetyButton->setText(QString("Safety Toggle: canSendControl -> OFF"));
        connect(safetyButton, &QPushButton::released, this, &ROARPanel::handleSafetyButtonTapped);
        main_layout->addWidget(safetyButton);

        emergencyBrakeButton = new QPushButton;
        emergencyBrakeButton->setText(QString("[NOT IMPLEMENTED]emergencyBrake: OFF"));
        connect(emergencyBrakeButton, &QPushButton::released, this, &ROARPanel::handleEmergencyBrakeButtonTapped);
        main_layout->addWidget(emergencyBrakeButton);

        main_layout->setContentsMargins(10, 10, 10, 10);
        setLayout(main_layout);
    }
    void ROARPanel::onInitialize()
    {
    }

    void
    ROARPanel::save(rviz_common::Config config) const
    {
        Panel::save(config);
    }

    void
    ROARPanel::load(const rviz_common::Config &config)
    {
        Panel::load(config);
    }

    void
    ROARPanel::handleSafetyButtonTapped()
    {
        this->canSendControl = !this->canSendControl;
        auto request = std::make_shared<roar_msgs::srv::ToggleControlSafetySwitch::Request>();
        request->is_safety_on = this->canSendControl;
        auto result = client->async_send_request(request);
        safetyButton->setText(QString("Safety Toggle: canSendControl -> ") + (this->canSendControl ? "YES" : "NO"));
    }

    void
    ROARPanel::handleEmergencyBrakeButtonTapped()
    {
        this->emergencyBrakeOn = !this->emergencyBrakeOn;
        emergencyBrakeButton->setText(QString("[NOT IMPLEMENTED]emergencyBrake:") + (this->emergencyBrakeOn ? "ON" : "OFF"));

        std::cout << "NOT IMPLEMNTED" << std::endl;
    }
} // namespace roar_rviz_plugin
#include <pluginlib/class_list_macros.hpp> // NOLINT
PLUGINLIB_EXPORT_CLASS(roar_rviz_plugin::ROARPanel, rviz_common::Panel)