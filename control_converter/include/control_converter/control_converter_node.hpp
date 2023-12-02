#ifndef CONTROL_CONVERTER_NODE_HPP_
#define CONTROL_CONVERTER_NODE_HPP_
#include "nav2_util/lifecycle_node.hpp"
#include "control_msgs/msg/pid_state.hpp"
#include "roar_gokart_msgs/msg/ego_vehicle_control.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
namespace control_converter
{
    class ControlConverterNode : public nav2_util::LifecycleNode
    {
    public:
        ControlConverterNode();

    protected:
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        void onLatestAckermannRcvd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
        float map_range(float value, float orig_min, float orig_max, float target_min, float target_max)
        {
            // calculate the original range
            float orig_range = orig_max - orig_min;

            // calculate the target range
            float target_range = target_max - target_min;

            // scale the original value to the target range
            return (((value - orig_min) * target_range) / orig_range) + target_min;
        }
        float ackermann_to_carla_steering(float ackermann_steering);

        rclcpp::Publisher<roar_gokart_msgs::msg::EgoVehicleControl>::SharedPtr control_publisher_;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_sub_;

        float GOKART_MAX_RIGHT_STEERING = 1.0;
        float GOKART_MAX_LEFT_STEERING = -1.0;

        float INPUT_MAX_RIGHT_STEERING = 1.0;
        float INPUT_MAX_LEFT_STEERING = 1.0;
    };
}

#endif // CONTROL_CONVERTER_NODE_HPP_
