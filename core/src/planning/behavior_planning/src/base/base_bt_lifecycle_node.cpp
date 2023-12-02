#include "behavior_planning/base/base_bt_lifecycle_node.hpp"
#include "behavior_planning/common/utils.hpp"

#include "behavior_planning/bt_nodes/condition_nodes/if_goal_reached.hpp"
#include "behavior_planning/bt_nodes/action_nodes/stop_car.hpp"
#include "behavior_planning/bt_nodes/action_nodes/lat_pid_tuner.hpp"
#include "roar_msgs/msg/behavior_status.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior_planning
        {
            namespace base
            {
                BehaviorPlannerBTLifeCycleNode::BehaviorPlannerBTLifeCycleNode(const rclcpp::NodeOptions &options) : BehaviorPlannerBaseLifecycleNode(options)
                {
                    this->declare_parameter("bt_xml_path", "./src/core/core/src/planning/behavior_planning/param/default.yml");
                    bt_xml_path_ = this->get_parameter("bt_xml_path").as_string();
                    RCLCPP_INFO_STREAM(get_logger(), "BehaviorPlannerBTLifeCycleNode is now online. bt_xml_path_: [" << bt_xml_path_ << "]");
                }

                void BehaviorPlannerBTLifeCycleNode::Initialize()
                {
                    RCLCPP_DEBUG(this->get_logger(), "Initialized BT Base Node");
                    blackboard_ = BT::Blackboard::create();
                    RegisterTreeNodes();
                    RCLCPP_INFO_STREAM(this->get_logger(), "Loading BT XML from: [" << bt_xml_path_ << "]");
                    bt_tree_ = factory_.createTreeFromFile(bt_xml_path_, blackboard_);
                    std::string groot_ip = "192.168.50.200";
                    groot_monitor_ =
                        std::make_unique<BT::PublisherZMQ>(
                            bt_tree_,
                            static_cast<unsigned>(100),
                            static_cast<unsigned>(1666),
                            static_cast<unsigned>(1667));
                    groot_log_file_ = std::make_unique<BT::FileLogger>(bt_tree_, "bt_trace.fbl");
                }
                bool BehaviorPlannerBTLifeCycleNode::on_step()
                {
                    RCLCPP_DEBUG(this->get_logger(), "Stepping BT Base Node");
                    // init output
                    blackboard_->set<roar::planning::behavior::BTOutputs::SharedPtr>("outputs", std::make_shared<roar::planning::behavior::BTOutputs>());

                    // TODO: update blackboard from params

                    // TODO: update inputs
                    blackboard_->set<const roar::planning::behavior::BTInputs::ConstSharedPtr>(
                        "inputs", GetInputs());

                    // tick tree
                    RunTree();

                    // post run tree

                    PostRunTree();

                    // get output
                    BT::Optional<roar::planning::behavior::BTOutputs::SharedPtr> outputs = blackboard_->get<roar::planning::behavior::BTOutputs::SharedPtr>("outputs");
                    if (!outputs)
                    {
                        RCLCPP_ERROR(this->get_logger(), "BehaviorPlannerBTLifeCycleNode: no outputs");
                        return false;
                    }

                    BehaviorPlannerBaseLifecycleNode::PublishBehaviorStatus(std::make_shared<roar_msgs::msg::BehaviorStatus>(outputs.value()->behavior_status));
                    return true;
                }

                BT::Blackboard::Ptr &BehaviorPlannerBTLifeCycleNode::GetBlackboard()
                {
                    return blackboard_;
                }

                BT::Tree &BehaviorPlannerBTLifeCycleNode::GetBtTree()
                {
                    return bt_tree_;
                }

                void BehaviorPlannerBTLifeCycleNode::RegisterTreeNodes()
                {
                    RCLCPP_INFO(this->get_logger(), "Registering Tree Nodes");
                    RegisterTreeNodeLogClock<roar::planning::behavior::condition::IfGoalReached>("IfGoalReached");

                    RegisterTreeNodeLogClock<roar::planning::behavior::action::StopCar>("StopCar");

                    RegisterTreeNodeLogClockWithNode<roar::planning::behavior::action::LatPIDtuner>("LatPIDtuner");
                }

                template <typename T>
                void BehaviorPlannerBTLifeCycleNode::RegisterTreeNodeLogClock(const std::string &node_name)
                {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Registering Tree Node: [" << node_name << "]");
                    BT::NodeBuilder builder =
                        [&](const std::string &name, const BT::NodeConfiguration &config)
                    {
                        return std::make_unique<T>(
                            name, config, this->get_logger(), *this->get_clock());
                    };
                    factory_.registerBuilder<T>(
                        node_name,
                        builder);
                }

                template <typename T>
                void BehaviorPlannerBTLifeCycleNode::RegisterTreeNodeLogClockWithNode(const std::string &node_name)
                {
                    RCLCPP_DEBUG_STREAM(this->get_logger(), "Registering Tree Node: [" << node_name << "]");
                    BT::NodeBuilder builder =
                        [&](const std::string &name, const BT::NodeConfiguration &config)
                    {
                        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> sharedPtr(this);

                        return std::make_unique<T>(
                            name, config, this->get_logger(), *this->get_clock(), sharedPtr);
                    };
                    factory_.registerBuilder<T>(
                        node_name,
                        builder);
                }
            } // namespace base
        }     // namespace behavior_planning
    }         // namespace behavior_planning
} // namespace roar
