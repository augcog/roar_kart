#ifndef ROAR__CONTROL__PLUGIN__LON_TRAPEZOIDAL_CONTROL_HPP_
#define ROAR__CONTROL__PLUGIN__LON_TRAPEZOIDAL_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "roar_msgs/msg/vehicle_state.hpp"
#include "controller_manager/controller_state.hpp"
using namespace roar::control;
namespace roar
{
    namespace control
    {
        struct LonTrapezoidalControllerPluginConfig
        {
            double accel_rate = 0.5; // m/s^2
            double decel_rate = 0.5; // m/s^2
            double target_speed = 5; // m/s

            double accel_time = 0.0;
            double accel_dist = 0.0;
            double decel_time = 0.0;
            double decel_dist = 0.0;
        };

        struct LonTrapezoidalControllerPluginState
        {
            roar_msgs::msg::VehicleState::SharedPtr vehicle_state;

            double last_compute_time = 0.0;

            double prev_spd = 0.0;
        };

        class LonTrapezoidalControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<LonTrapezoidalControllerPlugin> SharedPtr;
            typedef std::unique_ptr<LonTrapezoidalControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "LonTrapezoidalControllerPlugin";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
                auto defaultConfig = LonTrapezoidalControllerPluginConfig();
                config_ = LonTrapezoidalControllerPluginConfig{
                    this->node().declare_parameter<double>("lon_control.trapezoidal.accel_rate", defaultConfig.accel_rate),
                    this->node().declare_parameter<double>("lon_control.trapezoidal.decel_rate", defaultConfig.decel_rate),
                    this->node().declare_parameter<double>("lon_control.trapezoidal.target_speed", defaultConfig.target_speed)};
            }

            bool configure(const ControllerManagerConfig::SharedPtr config)
            {
                this->config_.accel_time = this->config_.target_speed / this->config_.accel_rate;
                this->config_.accel_dist = this->config_.target_speed * this->config_.accel_time / 2;
                this->config_.decel_time = this->config_.target_speed / this->config_.decel_rate;
                this->config_.decel_dist = this->config_.target_speed * this->config_.decel_time / 2;

                RCLCPP_INFO_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                            << "target_speed: " << this->config_.target_speed);

                RCLCPP_INFO_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                            << "accel_rate: " << this->config_.accel_rate);
                RCLCPP_INFO_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                            << "decel_rate: " << this->config_.decel_rate);

                RCLCPP_INFO_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                            << "accel_time: " << this->config_.accel_time);
                RCLCPP_INFO_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                            << "accel_dist: " << this->config_.accel_dist);
                RCLCPP_INFO_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                            << "decel_time: " << this->config_.decel_time);
                RCLCPP_INFO_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                            << "decel_dist: " << this->config_.decel_dist);

                return true;
            }

            bool update(const ControllerManagerState::SharedPtr state)
            {
                this->state_.vehicle_state = state->vehicle_state;
                return true;
            }
            bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg)
            {
                if (this->state_.vehicle_state == nullptr)
                {
                    RCLCPP_ERROR_STREAM(node().get_logger(), "vehicle_state is null");
                    return false;
                }

                double dt = node().now().seconds() + 1e-9 * node().now().nanoseconds() - this->state_.last_compute_time;

                // this->state_.vehicle_state
                geometry_msgs::msg::PoseStamped lastPose = this->state_.vehicle_state->global_path.poses[this->state_.vehicle_state->global_path.poses.size() - 1];

                double dist = std::sqrt(std::pow(lastPose.pose.position.x - this->state_.vehicle_state->odometry.pose.pose.position.x, 2) +
                                        std::pow(lastPose.pose.position.y - this->state_.vehicle_state->odometry.pose.pose.position.y, 2));

                if (dist < this->config_.decel_dist)
                {
                    // decelerate

                    double time_left = dist * 2 / this->config_.target_speed;
                    controlMsg->target_speed = this->config_.decel_rate * time_left;
                    RCLCPP_DEBUG_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                                 << " decelerating->"
                                                                 << "dist: [" << dist << "] decel_dist: ["
                                                                 << this->config_.decel_dist << "] setting target speed to: " << controlMsg->target_speed);
                    return true;
                }
                // accelerate until reaching target spd
                controlMsg->target_speed = std::min(this->state_.prev_spd + this->config_.accel_rate * dt, this->config_.target_speed);
                RCLCPP_DEBUG_STREAM(node().get_logger(), "[LonTrapezoidalController]: "
                                                             << "accelerating -> setting target speed to: " << controlMsg->target_speed);
                this->state_.prev_spd = controlMsg->target_speed;

                this->state_.last_compute_time = node().now().seconds() + 1e-9 * node().now().nanoseconds();
                return true;
            }

        private:
            LonTrapezoidalControllerPluginConfig config_;
            LonTrapezoidalControllerPluginState state_;
        };
    }
} // namespace roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::LonTrapezoidalControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR__CONTROL__PLUGIN__LON_TRAPEZOIDAL_CONTROL_HPP_