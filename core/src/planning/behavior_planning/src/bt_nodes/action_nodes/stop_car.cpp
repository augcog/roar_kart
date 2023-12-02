#include "behavior_planning/bt_nodes/action_nodes/stop_car.hpp"
#include "rclcpp/logging.hpp"
#include "behavior_planning/common/utils.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            namespace action
            {
                StopCar::StopCar(
                    const std::string &action_name,
                    const BT::NodeConfiguration &conf,
                    const rclcpp::Logger &logger,
                    rclcpp::Clock &clock) : BT::SyncActionNode(action_name, conf), logger_(logger), clock_(clock)
                {
                    RCLCPP_DEBUG(logger_, "StopCar created");
                }

                BT::NodeStatus StopCar::tick()
                {
                    RCLCPP_DEBUG(logger_, "StopCar actuating...");
                    BT::Optional<roar::planning::behavior::BTOutputs::SharedPtr> outputs = config().blackboard->get<roar::planning::behavior::BTOutputs::SharedPtr>("outputs");
                    if (!outputs)
                    {
                        RCLCPP_ERROR(logger_, "BehaviorPlannerBTLifeCycleNode: no outputs");
                        return BT::NodeStatus::FAILURE;
                    }
                    
                    outputs.value()->behavior_status.action_type = roar_msgs::msg::BehaviorStatus::ACTION_STOP;
                    return BT::NodeStatus::SUCCESS;
                }

                BT::PortsList StopCar::providedPorts()
                {
                    return {
                        BT::InputPort<roar::planning::behavior::BTInputs::ConstSharedPtr>("inputs"),
                        BT::OutputPort<roar::planning::behavior::BTOutputs::SharedPtr>("outputs")};
                }

            }
        }
    }
} // namespace roar
