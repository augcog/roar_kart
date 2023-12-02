#ifndef LOCAL_PLANNER_MANAGER_NODE_HPP_
#define LOCAL_PLANNER_MANAGER_NODE_HPP_

#include "control_interfaces/action/control.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <nav2_msgs/msg/costmap_meta_data.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav2_msgs/srv/get_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "local_planner_manager/local_planner_manager_state.hpp"
#include "local_planner_manager/local_planner_plugin_interface.hpp"
#include <pluginlib/class_loader.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

namespace roar
{
    namespace planning
    {
        namespace local
        {
            class LocalPlannerManagerNode : public nav2_util::LifecycleNode
            {
                using ControlAction = control_interfaces::action::Control;
                using GoalHandleControlAction =
                    rclcpp_action::ClientGoalHandle<ControlAction>;

            public:
                LocalPlannerManagerNode();
                ~LocalPlannerManagerNode();

                void execute();
            private:
                roar::planning::local::LocalPlannerManagerConfig::SharedPtr m_config_;
                rclcpp::TimerBase::SharedPtr execution_timer;

                // plugins
                typedef std::vector<LocalPlannerPlugin::SharedPtr> PluginList;
                pluginlib::ClassLoader<LocalPlannerPlugin> m_plugin_loader_;
                PluginList m_plugins_;
                LocalPlannerManagerState::SharedPtr m_state_;

            protected:
                // implement the lifecycle interface
                nav2_util::CallbackReturn
                on_configure(const rclcpp_lifecycle::State &state) override;

                nav2_util::CallbackReturn
                on_activate(const rclcpp_lifecycle::State &state) override;

                nav2_util::CallbackReturn
                on_deactivate(const rclcpp_lifecycle::State &state) override;

                nav2_util::CallbackReturn
                on_cleanup(const rclcpp_lifecycle::State &state) override;

                nav2_util::CallbackReturn
                on_shutdown(const rclcpp_lifecycle::State &state) override;

                /* execution */
                void p_execute();
                rclcpp::TimerBase::SharedPtr execute_timer;
                int num_execution = 0;
                bool didReceiveAllMessages();
                bool canExecute();

                // path picking
                std::string findBestPath(const std::map<std::string, nav_msgs::msg::Path::SharedPtr> pathsMap);

                /* footprint */
                rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::ConstSharedPtr
                    footprint_sub_;
                std::mutex footprint_mutex;
                void onLatestFootprintReceived(
                    geometry_msgs::msg::PolygonStamped::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(footprint_mutex);
                    this->m_state_->robot_footprint = msg;
                }

                /* Odometry */
                std::shared_ptr<nav_msgs::msg::Odometry> latest_odom;
                rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;
                std::mutex odom_mutex_;
                void onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(odom_mutex_);
                    this->m_state_->odom = msg;
                }

                /* occupancy map */
                rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
                    occupancy_map_sub_;
                std::mutex occupancy_map_mutex_;
                void onLatestOccupancyMapReceived(
                    nav_msgs::msg::OccupancyGrid::SharedPtr msg)
                {
                    std::lock_guard<std::mutex> lock(occupancy_map_mutex_);
                    this->m_state_->occupancy_map = msg;
                }
                                

                /* control client */
                rclcpp_action::Client<ControlAction>::SharedPtr control_action_client_;
                std::string controllerServerRoute;

                void control_action_goal_response_callback(
                    std::shared_future<GoalHandleControlAction::SharedPtr> future);
                void control_action_feedback_callback(
                    GoalHandleControlAction::SharedPtr future,
                    const std::shared_ptr<const ControlAction::Feedback> feedback);
                void control_action_result_callback(
                    const GoalHandleControlAction::WrappedResult &result);
                void control_send_goal(const nav_msgs::msg::Path::SharedPtr path,
                                       float target_spd);

                // diagnostic
                std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>> diagnostic_pub_;

                // global plan
                rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_plan_sub_;
                std::mutex global_plan_mutex_;
                void onLatestGlobalPlanReceived(nav_msgs::msg::Path::SharedPtr msg)
                {
                    if (msg->poses.size() == 0)
                    {
                        RCLCPP_WARN(get_logger(), "Global plan is empty, not accepting it.");
                        return;
                    }
                    std::lock_guard<std::mutex> lock(global_plan_mutex_);
                    this->m_state_->global_plan = msg;
                }

                // best path publisher
                std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> best_path_vis_marker_pub_;
                void publishBestPathVisualizationMarker(const nav_msgs::msg::Path::SharedPtr best_path)
                {
                    visualization_msgs::msg::MarkerArray marker_array;

                    for (int i = 0; i < best_path->poses.size(); i++)
                    {
                        geometry_msgs::msg::Point point;
                        point.x = best_path->poses[i].pose.position.x;
                        point.y = best_path->poses[i].pose.position.y;
                        point.z = best_path->poses[i].pose.position.z;

                        marker_array.markers.push_back(visualization_msgs::msg::Marker());
                        visualization_msgs::msg::Marker &marker = marker_array.markers.back();
                        marker.header.frame_id = "map";
                        marker.header.stamp = this->now();
                        marker.ns = "best_path";
                        marker.id = i;
                        marker.type = visualization_msgs::msg::Marker::SPHERE;
                        marker.action = visualization_msgs::msg::Marker::ADD;
                        marker.pose.position = point;
                        marker.pose.orientation.w = 1.0;
                        marker.scale.x = 0.2;
                        marker.scale.y = 0.2;
                        marker.scale.z = 0.2;
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        marker.color.a = 1.0;
                        marker.lifetime = rclcpp::Duration(0.1);
                    }

                    this->best_path_vis_marker_pub_->publish(marker_array);
                }
            };
        } 
    }
}
#endif