#ifndef ROAR__BEHAVIOR_PLANNING__CONDITION_NODES__IF_GOAL_REACHED_HPP_
#define ROAR__BEHAVIOR_PLANNING__CONDITION_NODES__IF_GOAL_REACHED_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            namespace condition
            {
                class IfGoalReached : public BT::ConditionNode
                {
                public:
                    IfGoalReached(
                        const std::string &condition_name,
                        const BT::NodeConfiguration &conf,
                        const rclcpp::Logger &logger,
                        rclcpp::Clock &clock);
                    BT::NodeStatus tick() override;
                    static BT::PortsList providedPorts();

                private:
                    rclcpp::Logger logger_;
                    rclcpp::Clock &clock_;
                };

            } // namespace condition
        }     // namespace behavior
    }         // namespace planning

} // namespace roar

#endif // ROAR__BEHAVIOR_PLANNING__CONDITION_NODES__IF_GOAL_REACHED_HPP_