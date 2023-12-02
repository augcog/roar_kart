#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "roar_gokart_msgs/msg/ego_vehicle_control.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class RoarGoKartConverter : public rclcpp::Node
{
public:
    RoarGoKartConverter()
        : Node("roar_gokart_converter")
    {
        control_subscriber = this->create_subscription<roar_msgs::msg::VehicleControl>(
            "roar_control", 10, std::bind(&RoarGoKartConverter::control_callback, this, std::placeholders::_1));

        control_publisher_ = this->create_publisher<roar_gokart_msgs::msg::EgoVehicleControl>("gokart_control", 10);
    }

private:
    void control_callback(const roar_msgs::msg::VehicleControl::SharedPtr msg)
    {
        roar_gokart_msgs::msg::EgoVehicleControl output_msg = roar_gokart_msgs::msg::EgoVehicleControl();
        output_msg.target_speed = msg->target_speed;
        output_msg.steering_angle = msg->steering_angle;
        output_msg.brake = msg->brake;
        output_msg.reverse = msg->reverse;

        control_publisher_->publish(output_msg);
    }

    rclcpp::Subscription<roar_msgs::msg::VehicleControl>::SharedPtr control_subscriber;
    rclcpp::Publisher<roar_gokart_msgs::msg::EgoVehicleControl>::SharedPtr control_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoarGoKartConverter>());
    rclcpp::shutdown();
    return 0;
}