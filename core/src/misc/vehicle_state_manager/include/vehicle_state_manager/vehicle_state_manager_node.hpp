#ifndef ROAR__VEHICLE_STATE_MANAGER_NODE_HPP_
#define ROAR__VEHICLE_STATE_MANAGER_NODE_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_state.hpp"
#include "roar_msgs/msg/vehicle_control.h"
#include "roar_msgs/msg/vehicle_status.hpp"
#include "roar_msgs/msg/behavior_status.hpp"
#include "nav_msgs/msg/path.hpp"
namespace roar
{
    class VehicleStateManagerNode : public nav2_util::LifecycleNode
    {
    public:
        VehicleStateManagerNode();
        ~VehicleStateManagerNode();

    protected:
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        roar_msgs::msg::VehicleState::SharedPtr vehicle_state = nullptr;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<roar_msgs::msg::VehicleState>> vehicle_state_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

        rclcpp::Subscription<roar_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
        void vehicle_status_callback(const roar_msgs::msg::VehicleStatus::SharedPtr msg);

        rclcpp::Subscription<roar_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
        void vehicle_state_callback(const roar_msgs::msg::VehicleState::SharedPtr msg);

        rclcpp::Subscription<roar_msgs::msg::VehicleControl>::SharedPtr vehicle_control_sub_;
        void vehicle_control_callback(const roar_msgs::msg::VehicleControl::SharedPtr msg);

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr next_waypoint_sub_;
        void next_waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        rclcpp::Subscription<roar_msgs::msg::BehaviorStatus>::SharedPtr behavior_status_sub_;
        void behavior_status_callback(const roar_msgs::msg::BehaviorStatus::SharedPtr msg);
        
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
        void global_path_callback(const nav_msgs::msg::Path::SharedPtr msg);
        
    private:
        rclcpp::TimerBase::SharedPtr update_timer;
        void update_callback();
    };
}

#endif // ROAR__VEHICLE_STATE_MANAGER_NODE_HPP_