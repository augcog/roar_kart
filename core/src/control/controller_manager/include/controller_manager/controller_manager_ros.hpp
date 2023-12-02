#ifndef CONTROLLER_MANAGER_ROS_HPP_
#define CONTROLLER_MANAGER_ROS_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_interfaces/action/control.hpp"
#include <nav2_util/lifecycle_node.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include "roar_msgs/msg/vehicle_control.hpp"
#include "roar_msgs/srv/toggle_control_safety_switch.hpp"
#include "controller_manager/controller_interface.hpp"
#include <tf2_ros/transform_listener.h>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <pluginlib/class_loader.hpp>
#include "roar_msgs/msg/behavior_status.hpp"

using namespace roar::control;

namespace controller
{
    enum Algorithms
    {
        PID,
    };

    class ControllerManagerNode : public nav2_util::LifecycleNode
    {
        using ControlAction = control_interfaces::action::Control;
        using GoalHandleControlAction = rclcpp_action::ServerGoalHandle<ControlAction>;

    public:
        ControllerManagerNode();
        ~ControllerManagerNode();

    protected:
        // implement the lifecycle interface
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        // execution
        void p_execute(const std::shared_ptr<GoalHandleControlAction> goal_handle);
        int num_execution = 0;
        bool canExecute(std::shared_ptr<const ControlAction::Goal> goal);
        void on_update();
        rclcpp::TimerBase::SharedPtr execution_timer;
        void execution_callback();

        /**
         * action server
         */
        rclcpp_action::Server<ControlAction>::SharedPtr action_server_;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ControlAction::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleControlAction> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleControlAction> goal_handle);
        std::shared_ptr<GoalHandleControlAction> active_goal_; // use this to ensure that only one goal is executing at a time
        std::mutex active_goal_mutex_;

        /**
         * control publisher
         */
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<roar_msgs::msg::VehicleControl>> vehicle_control_publisher_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostic_pub_;

        bool is_auto_control = false;

        rclcpp::Service<roar_msgs::srv::ToggleControlSafetySwitch>::SharedPtr control_safety_switch_;
        void toggle_safety_switch(const std::shared_ptr<roar_msgs::srv::ToggleControlSafetySwitch::Request> request,
                                  std::shared_ptr<roar_msgs::srv::ToggleControlSafetySwitch::Response> response);

        nav_msgs::msg::Path p_transformToEgoCentric(nav_msgs::msg::Path path);

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        // plugins
        typedef std::vector<roar::control::ControllerPlugin::SharedPtr> PluginList;
        pluginlib::ClassLoader<roar::control::ControllerPlugin> m_plugin_loader_;
        PluginList m_plugins_;

        roar::control::ControllerManagerConfig::SharedPtr m_config_;

        // interaction with behavior planning module
        rclcpp::Subscription<roar_msgs::msg::BehaviorStatus>::SharedPtr behavior_status_sub_;
        void behavior_status_callback(const roar_msgs::msg::BehaviorStatus::SharedPtr msg);

        // vehicle state listener
        rclcpp::Subscription<roar_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_;
        void vehicle_state_callback(const roar_msgs::msg::VehicleState::SharedPtr msg)
        {
            m_controller_state_->vehicle_state = msg;
        }

    private:
        ControllerManagerState::SharedPtr m_controller_state_;
    };
} // controller

#endif