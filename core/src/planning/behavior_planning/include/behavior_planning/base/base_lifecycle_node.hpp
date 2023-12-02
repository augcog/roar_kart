// Copyright 2023 Siddharth Saha

#ifndef ROAR__BEHAVIOR_PLANNING__BASE_LIFECYCLE_NODE_HPP_
#define ROAR__BEHAVIOR_PLANNING__BASE_LIFECYCLE_NODE_HPP_

#include <memory>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "behavior_planning/common/utils.hpp"
#include "roar_msgs/msg/behavior_status.hpp"
#include "roar_msgs/msg/vehicle_state.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior_planning
        {
            namespace base
            {
                class BehaviorPlannerBaseLifecycleNode : public rclcpp_lifecycle::LifecycleNode
                {
                public:
                    explicit BehaviorPlannerBaseLifecycleNode(const rclcpp::NodeOptions &options);
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
                    on_configure(
                        const rclcpp_lifecycle::State &previous_state) override;
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
                        const rclcpp_lifecycle::State &previous_state) override;
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
                        const rclcpp_lifecycle::State &previous_state) override;
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
                        const rclcpp_lifecycle::State &previous_state) override;
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
                        const rclcpp_lifecycle::State &previous_state) override;
                    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
                        const rclcpp_lifecycle::State &previous_state) override;

                protected:
                    virtual void Initialize() = 0;
                    void on_timer_callback();
                    virtual bool on_step() = 0;

                    // accessor methods
                    const roar::planning::behavior::BTInputs::ConstSharedPtr GetInputs();

                    // tree methods
                    void PublishBehaviorStatus(const roar_msgs::msg::BehaviorStatus::SharedPtr behavior_status);

                private:
                    rclcpp::TimerBase::SharedPtr timer_{};
                    double loop_rate_ = 0.1;

                    roar::planning::behavior::BTInputs::SharedPtr bt_inputs_{};

                    // publisher
                    rclcpp_lifecycle::LifecyclePublisher<roar_msgs::msg::BehaviorStatus>::SharedPtr behavior_status_pub_{};

                    // subscriber
                    rclcpp::Subscription<roar_msgs::msg::VehicleState>::SharedPtr vehicle_state_sub_{};
                    void vehicle_state_callback(const roar_msgs::msg::VehicleState::SharedPtr msg);
                };
            } // namespace base
        }     // namespace behavior_planning
    }         // namespace planning
} // namespace roar

#endif // ROAR__BEHAVIOR_PLANNING__BASE_LIFECYCLE_NODE_HPP_
