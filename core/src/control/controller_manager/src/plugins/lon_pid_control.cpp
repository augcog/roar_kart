#ifndef ROAR_CONTROL__PLUGIN__LONG_PID_CONTROL_HPP_
#define ROAR_CONTROL__PLUGIN__LONG_PID_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "controller_manager/controller_state.hpp"
#include "controller_manager/pid_controller.hpp"

using namespace roar::control;
namespace roar
{
    namespace control
    {
        struct LonConfig
        {
            PidCoefficients throttle_pid_param;
        };

        struct LonControlState
        {
            double target_speed = 2.0; // modify this to set default target speed
            double current_speed = 0.0;
            double throttle_error = 0.0;
            double speed_output = 0.0;
            PidController throttle_pid;
            rclcpp::Time last_pid_time;
            rclcpp::TimerBase::SharedPtr pid_timer;
        };

        class LongPIDControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<LongPIDControllerPlugin> SharedPtr;
            typedef std::unique_ptr<LongPIDControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "LongPIDControllerPlugin";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
                config_ = LonConfig{
                    PidCoefficients{
                        this->node().declare_parameter<double>("lon_control.pid.kp", 1.0),
                        this->node().declare_parameter<double>("lon_control.pid.ki", 0.1),
                        this->node().declare_parameter<double>("lon_control.pid.kd", 0.1),
                        this->node().declare_parameter<double>("lon_control.pid.min_cmd", 0.0),
                        this->node().declare_parameter<double>("lon_control.pid.max_cmd", 20.0),
                        this->node().declare_parameter<double>("lon_control.pid.min_i", -10.0),
                        this->node().declare_parameter<double>("lon_control.pid.max_i", 10.0),
                    }};
            }

            bool configure(const ControllerManagerConfig::SharedPtr config)
            {
                lon_state().throttle_pid = PidController("throttle", config_.throttle_pid_param);
                return true;
            }
            bool update(const ControllerManagerState::SharedPtr state)
            {
                /**
                 * WARNING: this file is not completed. DO NOT USE IT.
                 */
                // update PID State from ControllerState
                // lon_state().current_speed = state->current_speed;

                // compute the throttle cmd
                static auto this_pid_time = node().now();
                const auto dt = (this_pid_time - lon_state().last_pid_time).seconds();
                const auto speed_error = lon_state().target_speed - lon_state().current_speed;
                RCLCPP_INFO_STREAM(node().get_logger(), "speed_error: " << speed_error << " dt: " << dt);
                double speed_output = state_.throttle_pid.update(speed_error, dt);

                // update the state
                lon_state().throttle_error = speed_error;
                lon_state().speed_output = speed_output;
                lon_state().last_pid_time = this_pid_time;
                return true;
            }
            bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg)
            {
                controlMsg->target_speed = lon_state().speed_output;
                return true;
            }

        private:
            LonConfig config_;
            LonControlState state_;

        protected:
            LonControlState &lon_state()
            {
                return state_;
            }
        };
    } // namespace control
} // roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::LongPIDControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR_CONTROL__PLUGIN__LONG_PID_CONTROL_HPP_