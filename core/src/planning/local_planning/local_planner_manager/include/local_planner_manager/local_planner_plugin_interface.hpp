#ifndef ROAR_PLANNING__PLUGIN_HPP_
#define ROAR_PLANNING__PLUGIN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "local_planner_manager/local_planner_manager_state.hpp"
#include "nav_msgs/msg/path.hpp"

namespace roar
{
    namespace planning
    {
        namespace local
        {
            class LocalPlannerPlugin
            {
            public:
                typedef std::shared_ptr<LocalPlannerPlugin> SharedPtr;
                typedef std::unique_ptr<LocalPlannerPlugin> UniquePtr;

                virtual ~LocalPlannerPlugin() = default;
                virtual void initialize(nav2_util::LifecycleNode *node) { node_ = node; }
                virtual bool configure(const LocalPlannerManagerConfig::SharedPtr config) { return true; }
                virtual bool update(const LocalPlannerManagerState::SharedPtr state) { return true; }
                virtual nav_msgs::msg::Path::SharedPtr compute() = 0;
                virtual const char *get_plugin_name()
                {
                    return "default plugin name";
                }

            protected:
                nav2_util::LifecycleNode &node() { return *node_; }

            private:
                nav2_util::LifecycleNode *node_{};
            };
        } // namespace local
    }     // namespace planning

} // namespace roar

#endif // ROAR_PLANNING__PLUGIN_HPP_