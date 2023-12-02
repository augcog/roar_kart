#include "behavior_planning/bt_nodes/condition_nodes/if_goal_reached.hpp"
#include "rclcpp/logging.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "behavior_planning/common/utils.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            namespace condition
            {
                IfGoalReached::IfGoalReached(
                    const std::string &condition_name,
                    const BT::NodeConfiguration &conf,
                    const rclcpp::Logger &logger,
                    rclcpp::Clock &clock) : BT::ConditionNode(condition_name, conf), logger_(logger), clock_(clock)
                {
                    RCLCPP_DEBUG(logger_, "IfGoalReached created");
                }

                BT::NodeStatus IfGoalReached::tick()
                {
                    RCLCPP_DEBUG(logger_, "IfGoalReached ticked");

                    // const std::shared_ptr<roar::planning::behavior::BTInputs> inputs =
                    auto inputs = config().blackboard->get<roar::planning::behavior::BTInputs::ConstSharedPtr>("inputs");
                    if (!inputs)
                    {
                        RCLCPP_ERROR(logger_, "IfGoalReached: no inputs");
                        return BT::NodeStatus::FAILURE;
                    }

                    if (!inputs->vehicle_state)
                    {
                        RCLCPP_WARN(logger_, "IfGoalReached: no vehicle state");
                        return BT::NodeStatus::FAILURE;
                    }
                    if (inputs->vehicle_state->global_path.poses.empty())
                    {
                        // user have not chosen path yet probably
                        return BT::NodeStatus::FAILURE;
                    }

                    auto goal = inputs->vehicle_state->global_path.poses.back();
                    auto odom = inputs->vehicle_state->odometry;
                    BT::Optional<std::string> goal_radius_raw = getInput<std::string>("goal_radius");
                    if (!goal_radius_raw)
                    {
                        RCLCPP_ERROR(logger_, "IfGoalReached: goal_radius is not specified");
                    }

                    float goal_radius = std::stof(goal_radius_raw.value());
                    if (goal_radius <= 0)
                    {
                        RCLCPP_ERROR(logger_, "IfGoalReached: goal_radius cannot be <= 0");
                        return BT::NodeStatus::FAILURE;
                    }

                    // if within a certain radius of the goal, return success
                    float x1, y1, x2, y2;
                    x1 = odom.pose.pose.position.x;
                    y1 = odom.pose.pose.position.y;
                    x2 = goal.pose.position.x;
                    y2 = goal.pose.position.y;

                    RCLCPP_DEBUG_STREAM(logger_, "x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2);

                    double distance = std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
                    if (distance < 0)
                    {
                        RCLCPP_ERROR(logger_, "IfGoalReached: distance cannot be  < 0");
                        return BT::NodeStatus::FAILURE;
                    }
                    else if (distance == 0)
                    {
                        // cannot be 0
                        return BT::NodeStatus::FAILURE;
                    }

                    else if (distance < goal_radius)
                    {
                        RCLCPP_INFO_STREAM(logger_, "Goal Reached");
                        return BT::NodeStatus::SUCCESS;
                    }
                    RCLCPP_DEBUG_STREAM(logger_, "Not reaching goal: distance: " << distance << " goal_radius: " << goal_radius);
                    return BT::NodeStatus::FAILURE;
                }

                BT::PortsList IfGoalReached::providedPorts()
                {
                    return {
                        BT::InputPort<std::string>("goal_radius"),
                        BT::InputPort<roar::planning::behavior::BTInputs::ConstSharedPtr>("inputs"),
                    };
                }

            } // namespace condition
        }     // namespace behavior
    }         // namespace planning
} // namespace roar