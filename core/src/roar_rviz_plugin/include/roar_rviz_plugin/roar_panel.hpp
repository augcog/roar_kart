#ifndef ROAR_RVIZ_PLUGINS__ROAR_PANEL_HPP_
#define ROAR_RVIZ_PLUGINS__ROAR_PANEL_HPP_

#include <QtWidgets>
#include <QBasicTimer>
#undef NO_ERROR

#include <memory>
#include <string>
#include <vector>
#include "rviz_common/panel.hpp"
#include <nav2_lifecycle_manager/lifecycle_manager.hpp>
#include "roar_rviz_plugin/switch.hpp"
#include "roar_msgs/srv/toggle_control_safety_switch.hpp"
#include "rclcpp/rclcpp.hpp"

class QPushButton;

namespace roar_rviz_plugin
{
    class ROARPanel : public rviz_common::Panel
    {
    public:
        explicit ROARPanel(QWidget *parent = 0);
        virtual ~ROARPanel();

        void onInitialize() override;

        /// Load and save configuration data
        void load(const rviz_common::Config &config) override;
        void save(rviz_common::Config config) const override;

    private:
        void setupUI();

        QPushButton *safetyButton{nullptr};
        void handleSafetyButtonTapped();
        bool canSendControl = false;
        std::shared_ptr<rclcpp::Node> node;
        rclcpp::Client<roar_msgs::srv::ToggleControlSafetySwitch>::SharedPtr client;

        QPushButton *emergencyBrakeButton{nullptr};
        void handleEmergencyBrakeButtonTapped();
        bool emergencyBrakeOn = false;
        // control_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
    };
}
#endif //  ROAR_RVIZ_PLUGINS__ROAR_PANEL_HPP_
