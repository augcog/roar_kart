#ifndef BEHAVIOR_PLANNER_MANAGER_HPP
#define BEHAVIOR_PLANNER_MANAGER_HPP

#include "nav2_util/lifecycle_node.hpp"

namespace ROAR
{
    namespace BehaviorPlanning
    {
        class BehaviorPlannerManager : public nav2_util::LifecycleNode
        {
        public:
            using LifecycleNode::LifecycleNode;
            BehaviorPlannerManager();
            ~BehaviorPlannerManager();

        protected:
            // Implement the necessary lifecycle functions

            // This function is called when the node transitions from inactive to active state
            nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

            // This function is called when the node transitions from active to inactive state
            nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

            // This function is called when the node transitions from finalized to initialized state
            nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

            // This function is called when the node transitions from initialized to inactive state
            nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

            // This function is called when the node transitions from inactive to finalized state
            nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
        };
    } // namespace global_planning
} // namespace roar

#endif // BEHAVIOR_PLANNER_MANAGER_HPP