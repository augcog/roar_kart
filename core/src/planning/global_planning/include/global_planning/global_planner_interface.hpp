#ifndef GLOBAL_PLANNER_INTERFACE_HPP
#define GLOBAL_PLANNER_INTERFACE_HPP
#include <rclcpp/rclcpp.hpp>
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace ROAR
{
    namespace GlobalPlanning
    {
        struct StepInput
        {
            nav_msgs::msg::Odometry::SharedPtr odom;
        };

        struct StepResult
        {
            nav_msgs::msg::Path::SharedPtr global_path;

            bool status = true;
        };

        class GlobalPlannerInterface
        {
        public:
            GlobalPlannerInterface(nav2_util::LifecycleNode *node, std::string name = "GlobalPlannerInterface") :
            m_node_(node), m_logger_(rclcpp::get_logger(name))
            {
            }
            virtual ~GlobalPlannerInterface() = default;

            virtual void initialize() = 0;
            virtual bool on_configure()
            {
                return true;
            }
            virtual bool on_activate()
            {
                return true;
            }
            virtual bool on_deactivate()
            {
                return true;
            }
            virtual bool on_cleanup()
            {
                return true;
            }
            virtual bool on_shutdown()
            {
                return true;
            }

            virtual StepResult step(const StepInput input) = 0;

        protected:
            nav2_util::LifecycleNode *m_node_{};
            rclcpp::Logger m_logger_;
        };
    }
} // namespace roar

#endif // GLOBAL_PLANNER_INTERFACE_HPP