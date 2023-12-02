#ifndef GLOBAL_PLANNER_MANAGER_HPP
#define GLOBAL_PLANNER_MANAGER_HPP

#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "global_planning/global_planner_interface.hpp"

namespace ROAR
{
    namespace GlobalPlanning
    {
        class GlobalPlannerManager : public nav2_util::LifecycleNode
        {
        public:
            using LifecycleNode::LifecycleNode;
            GlobalPlannerManager();
            ~GlobalPlannerManager();

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

        private:
            GlobalPlannerInterface *planner{};
            rclcpp::TimerBase::SharedPtr timer{};
            void step();

            // odom subscriber
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_subscriber_;
            nav_msgs::msg::Odometry::SharedPtr current_odom;

            // publishers
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_path_publisher_;
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> global_path_visualization_publisher_;
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> vehicle_curr_position_publisher_;
        };
    } // namespace global_planning
} // namespace roar

#endif // GLOBAL_PLANNER_NODE_HPP