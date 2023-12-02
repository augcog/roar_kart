
#include "control_converter/control_converter_node.hpp"
using std::placeholders::_1;

namespace control_converter
{
    ControlConverterNode::ControlConverterNode()
        : LifecycleNode("control_converter_node", "", true)
    {
        this->declare_parameter("input_max_right_steering", 1.0);
        this->declare_parameter("input_max_left_steering", -1.0);

        this->INPUT_MAX_RIGHT_STEERING = this->get_parameter("input_max_right_steering").as_double();
        this->INPUT_MAX_LEFT_STEERING = this->get_parameter("input_max_left_steering").as_double();
        RCLCPP_INFO(get_logger(), "Creating control_converter_node");
    }

    nav2_util::CallbackReturn ControlConverterNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Configuring control_converter_node");

        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn ControlConverterNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Activating control_converter_node");
        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "ackermann_sub", 10, std::bind(&ControlConverterNode::onLatestAckermannRcvd, this, _1));
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn ControlConverterNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Deactivating control_converter_node");
        // on_deactivate implementation
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn ControlConverterNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up control_converter_node");

        // on_cleanup implementation
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn ControlConverterNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        // on_shutdown implementation
        return nav2_util::CallbackReturn::SUCCESS;
    }

    void ControlConverterNode::onLatestAckermannRcvd(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
        roar_gokart_msgs::msg::EgoVehicleControl ego_vehicle_msg = roar_gokart_msgs::msg::EgoVehicleControl();
        // ego_vehicle_msg.throttle = msg->drive.jerk;
        // ego_vehicle_msg.steer = msg->drive.steering_angle_velocity;
        // ego_vehicle_msg.brake = msg->drive.speed == 0;
        // ego_vehicle_msg.reverse = msg->drive.acceleration < 0;
        this->control_publisher_->publish(ego_vehicle_msg);
    }

    float ControlConverterNode::ackermann_to_carla_steering(float ackermann_steering)
    {
        return std::max(GOKART_MAX_LEFT_STEERING,
                        std::min(GOKART_MAX_RIGHT_STEERING,
                                 this->map_range(ackermann_steering,
                                                 GOKART_MAX_LEFT_STEERING,
                                                 GOKART_MAX_RIGHT_STEERING,
                                                 INPUT_MAX_LEFT_STEERING,
                                                 INPUT_MAX_RIGHT_STEERING)));
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<control_converter::ControlConverterNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}