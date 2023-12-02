#ifndef ROAR__BEHAVIOR_PLANNING__BASE_LIFECYCLE_BT_NODE_HPP_
#define ROAR__BEHAVIOR_PLANNING__BASE_LIFECYCLE_BT_NODE_HPP_

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

#include "behavior_planning/base/base_lifecycle_node.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior_planning
        {
            namespace base
            {
                class BehaviorPlannerBTLifeCycleNode : public BehaviorPlannerBaseLifecycleNode
                {
                public:
                    explicit BehaviorPlannerBTLifeCycleNode(const rclcpp::NodeOptions &options);

                protected:
                    // methods inherited from BehaviorPlannerBaseLifecycleNode
                    void Initialize() override;
                    bool on_step() override;

                    // methods to be overwritten from impl
                    virtual void RunTree() = 0;
                    virtual void PostRunTree() = 0;

                    BT::Blackboard::Ptr &GetBlackboard();
                    BT::Tree &GetBtTree();
                    void RegisterTreeNodes();

                private:
                    BT::BehaviorTreeFactory factory_;
                    BT::Blackboard::Ptr blackboard_;
                    BT::Tree bt_tree_;

                    std::string bt_xml_path_;

                    std::unique_ptr<BT::PublisherZMQ> groot_monitor_;
                    std::unique_ptr<BT::FileLogger> groot_log_file_;

                    template <typename T>
                    void RegisterTreeNodeLogClock(const std::string &node_name);

                    template <typename T>
                    void RegisterTreeNodeLogClockWithNode(const std::string &node_name);
                };
            }
        } // namespace behavior_planning

    }
} // namespace ROAR

#endif // ROAR__BEHAVIOR_PLANNING__BASE_LIFECYCLE_BT_NODE_HPP_