#ifndef ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_
#define ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_

#include "rclcpp/rclcpp.hpp"
#include "local_planner_manager/local_planner_manager_state.hpp"
#include "local_planner_manager/local_planner_plugin_interface.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

namespace roar
{
    namespace planning
    {
        namespace local
        {

            class SimpleDistancePlanner : public LocalPlannerPlugin
            {
                
            public:
                struct State
                {
                    nav_msgs::msg::Odometry::SharedPtr latest_odom;
                    nav_msgs::msg::Path::SharedPtr global_plan_;
                };

                struct Config
                {
                    double next_waypoint_dist;
                };

                SimpleDistancePlanner() = default;
                ~SimpleDistancePlanner() = default;

                void initialize(nav2_util::LifecycleNode *node) override
                {
                    RCLCPP_DEBUG(node->get_logger(), "Initializing SimpleDistancePlanner");
                    node_ = node;
                    this->m_state_ = std::make_shared<State>();
                    this->config_ = Config{
                        node_->declare_parameter<double>("SimpleDistancePlanner.next_waypoint_dist", 5.0),
                    };
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "next_waypoint_dist: " << this->config_.next_waypoint_dist);
                }

                const char *get_plugin_name() override
                {
                    return "SimpleDistancePlanner";
                }

                bool configure(const LocalPlannerManagerConfig::SharedPtr config) override
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Configuring SimpleDistancePlanner");
                    return true;
                }

                bool update(const LocalPlannerManagerState::SharedPtr state) override
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Updating SimpleDistancePlanner");
                    this->m_state_->latest_odom = state->odom;
                    this->m_state_->global_plan_ = state->global_plan;
                    return true;
                }

                nav_msgs::msg::Path::SharedPtr compute() override
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Computing SimpleDistancePlanner");
                    if (this->m_state_->latest_odom == nullptr)
                    {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), "latest_odom is null, not computing");
                        return nullptr;
                    }
                    if (this->m_state_->global_plan_ == nullptr)
                    {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), "global_plan_ is null, not computing");
                        return nullptr;
                    }

                    nav_msgs::msg::Path::SharedPtr path = this->findPathToNextWaypoint(this->config_.next_waypoint_dist, *this->m_state_->latest_odom, *this->m_state_->global_plan_);
                    if (path == nullptr || path->poses.size() == 0)
                    {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), "findPathToNextWaypoint yielded is null, unable to find path");
                        return nullptr;
                    }

                    return path;
                }

                nav_msgs::msg::Path::SharedPtr findPathToNextWaypoint(const float next_waypoint_dist, nav_msgs::msg::Odometry odom, nav_msgs::msg::Path global_plan)
                {
                    geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint = this->findNextWaypoint(next_waypoint_dist, odom, global_plan);
                    if (next_waypoint == nullptr)
                    {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), "findNextWaypoint yielded is null, unable to find path");
                        return nullptr;
                    }

                    nav_msgs::msg::Path::SharedPtr path = std::make_shared<nav_msgs::msg::Path>();
                    path->header = global_plan.header;
                    path->header.stamp = node_->now();

                    // generate n waypoints from odom to next_waypoint
                    int n = 10;
                    double dx = (next_waypoint->pose.position.x - odom.pose.pose.position.x) / n;
                    double dy = (next_waypoint->pose.position.y - odom.pose.pose.position.y) / n;
                    double dz = (next_waypoint->pose.position.z - odom.pose.pose.position.z) / n;

                    for (int i = 0; i < n; i++)
                    {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = odom.header;
                        pose.header.stamp = node_->now();
                        pose.pose.position.x = odom.pose.pose.position.x + dx * i;
                        pose.pose.position.y = odom.pose.pose.position.y + dy * i;
                        pose.pose.position.z = odom.pose.pose.position.z + dz * i;
                        path->poses.push_back(pose);
                    }
                    return path;
                }

                geometry_msgs::msg::PoseStamped::SharedPtr findNextWaypoint(const float next_waypoint_min_dist, nav_msgs::msg::Odometry odom, nav_msgs::msg::Path global_plan)
                {
                    RCLCPP_DEBUG(node_->get_logger(), "finding waypoint");

                    // find closest waypoint
                    // find the closest waypoint to the current position
                    double min_distance = std::numeric_limits<double>::max();
                    size_t closest_waypoint_index = 0;
                    for (size_t i = 0; i < global_plan.poses.size(); i++)
                    {
                        double distance = std::sqrt(std::pow(odom.pose.pose.position.x - global_plan.poses[i].pose.position.x, 2) +
                                                    std::pow(odom.pose.pose.position.y - global_plan.poses[i].pose.position.y, 2) +
                                                    std::pow(odom.pose.pose.position.z - global_plan.poses[i].pose.position.z, 2));
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                            closest_waypoint_index = i;
                        }
                    }
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "closest waypoint index: " << closest_waypoint_index << ", distance: " << min_distance);

                    // find the next waypoint, including looping back to the beginning
                    // double next_waypoint_dist = cte_and_lookahead.second;
                    double next_waypoint_dist = next_waypoint_min_dist;
                    size_t next_waypoint_index = closest_waypoint_index;
                    for (size_t i = 0; i < global_plan.poses.size(); i++)
                    {
                        size_t next_index = (closest_waypoint_index + i) % global_plan.poses.size();
                        double distance = std::sqrt(std::pow(odom.pose.pose.position.x - global_plan.poses[next_index].pose.position.x, 2) +
                                                    std::pow(odom.pose.pose.position.y - global_plan.poses[next_index].pose.position.y, 2) +
                                                    std::pow(odom.pose.pose.position.z - global_plan.poses[next_index].pose.position.z, 2));
                        if (distance > next_waypoint_dist)
                        {
                            next_waypoint_index = next_index;
                            break;
                        }
                    }
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "next waypoint index: " << next_waypoint_index << ", next_waypoint_dist: " << next_waypoint_dist);
                    geometry_msgs::msg::PoseStamped next_waypoint = global_plan.poses[next_waypoint_index];

                    return std::make_shared<geometry_msgs::msg::PoseStamped>(next_waypoint);
                }

            private:
                nav2_util::LifecycleNode *node_{};
                std::shared_ptr<State> m_state_;
                Config config_;
            };
        } // namespace local
    }     // namespace planning
} // namespace roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::planning::local::SimpleDistancePlanner, roar::planning::local::LocalPlannerPlugin)

#endif // ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_