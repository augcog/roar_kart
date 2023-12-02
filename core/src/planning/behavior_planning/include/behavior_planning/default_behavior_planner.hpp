#ifndef ROAR__IMPL__DEFAULT_BEHAVIOR_PLANNER_HPP_
#define ROAR__IMPL__DEFAULT_BEHAVIOR_PLANNER_HPP_

#include "behavior_planning/base/base_bt_lifecycle_node.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior_planning
        {
            namespace impl
            {
                class DefaultBehaviorPlanner : public base::BehaviorPlannerBTLifeCycleNode
                {
                public:
                    explicit DefaultBehaviorPlanner(const rclcpp::NodeOptions &options);
                    ~DefaultBehaviorPlanner() = default;

                private:
                    void Initialize() override;
                    void RunTree() override;
                    void PostRunTree() override;
                };
            } // namespace impl
        }     // namespace behavior_planning
    }         // namespace planning
} // namespace roar

#endif // ROAR__IMPL__DEFAULT_BEHAVIOR_PLANNER_HPP_