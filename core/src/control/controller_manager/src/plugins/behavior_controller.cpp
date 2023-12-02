#ifndef ROAR_CONTROL__PLUGIN__BEHAVIOR_CONTROL_HPP_
#define ROAR_CONTROL__PLUGIN__BEHAVIOR_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "controller_manager/controller_state.hpp"

#include "roar_msgs/msg/behavior_status.hpp"

using namespace roar::control;
namespace roar
{
    namespace control
    {
        class BehaviorControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<BehaviorControllerPlugin> SharedPtr;
            typedef std::unique_ptr<BehaviorControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "BehaviorControllerPlugin";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
            }

            bool configure(const ControllerManagerConfig::SharedPtr config) override
            {
                return true;
            }
            bool update(const ControllerManagerState::SharedPtr state) override
            {
                behavior_status_ = state->behavior_status;
                return true;
            }
            bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg)
            {
                if (behavior_status_ == nullptr)
                {
                    return true;
                }
                if (behavior_status_->action_type == roar_msgs::msg::BehaviorStatus::ACTION_GO ||
                    behavior_status_->action_type == roar_msgs::msg::BehaviorStatus::ACTION_UNKNOWN)
                {
                    return true;
                }

                if (behavior_status_->action_type == roar_msgs::msg::BehaviorStatus::ACTION_STOP)
                {
                    RCLCPP_INFO(node().get_logger(), "BehaviorControllerPlugin: stopping");
                    controlMsg->steering_angle = 0.0;
                    controlMsg->target_speed = 0.0;
                    controlMsg->brake = 1.0;
                    return true;
                }
                return true;
            }

        private:
            roar_msgs::msg::BehaviorStatus::SharedPtr behavior_status_;
        };
    } // namespace control
} // roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::BehaviorControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR_CONTROL__PLUGIN__BEHAVIOR_CONTROL_HPP_