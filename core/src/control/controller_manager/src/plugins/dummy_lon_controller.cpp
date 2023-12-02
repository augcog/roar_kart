#ifndef ROAR_CONTROL__PLUGIN__DUMMY_LON_PID_CONTROL_HPP_
#define ROAR_CONTROL__PLUGIN__DUMMY_LON_PID_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "controller_manager/controller_state.hpp"
using namespace roar::control;
namespace roar
{
    namespace control
    {
        struct LonConfig
        {
            double target_speed;
        };

        class DummyLonControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<DummyLonControllerPlugin> SharedPtr;
            typedef std::unique_ptr<DummyLonControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "DummyLonControllerPlugin";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
                config_ = LonConfig();
            }

            bool configure(const ControllerManagerConfig::SharedPtr config) override
            {
                config_.target_speed = config->target_speed;
                return true;
            }
            bool update(const ControllerManagerState::SharedPtr state) override
            {
                return true;
            }
            bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg)
            {
                controlMsg->target_speed = config_.target_speed; // target speed in m/s
                return true;
            }

        private:
            LonConfig config_;
        };
    } // namespace control
} // roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::DummyLonControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR_CONTROL__PLUGIN__DUMMY_LON_PID_CONTROL_HPP_