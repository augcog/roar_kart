#ifndef ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_
#define ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_

#include "rclcpp/rclcpp.hpp"
#include "local_planner_manager/local_planner_manager_state.hpp"
#include "local_planner_manager/local_planner_plugin_interface.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace roar
{
    namespace planning
    {
        namespace local
        {

            class SimpleFrenetFramePlanner : public LocalPlannerPlugin
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

                SimpleFrenetFramePlanner() = default;
                ~SimpleFrenetFramePlanner() = default;

                void initialize(nav2_util::LifecycleNode *node) override
                {
                    RCLCPP_DEBUG(node->get_logger(), "Initializing SimpleFrenetFramePlanner");
                    node_ = node;
                    this->m_state_ = std::make_shared<State>();
                    this->config_ = Config{
                        node_->declare_parameter<double>("SimpleFrenetFramePlanner.next_waypoint_dist", 5.0),
                    };

                    this->debug_vis_publisher_ = this->node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                        "/frenet_frame_debug_vis", rclcpp::SystemDefaultsQoS());

                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "next_waypoint_dist: " << this->config_.next_waypoint_dist);
                }

                const char *get_plugin_name() override
                {
                    return "SimpleFrenetFramePlanner";
                }

                bool configure(const LocalPlannerManagerConfig::SharedPtr config) override
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Configuring SimpleFrenetFramePlanner");
                    this->debug_vis_publisher_->on_activate();
                    return true;
                }

                bool update(const LocalPlannerManagerState::SharedPtr state) override
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Updating SimpleFrenetFramePlanner");
                    this->m_state_->latest_odom = state->odom;
                    this->m_state_->global_plan_ = state->global_plan;
                    return true;
                }

                nav_msgs::msg::Path::SharedPtr compute() override
                {
                    RCLCPP_DEBUG(node_->get_logger(), "Computing SimpleFrenetFramePlanner");
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

                    std::vector<nav_msgs::msg::Path::SharedPtr> possiblePaths = this->generatePossiblePaths();
                    if (possiblePaths.size() == 0)
                    {
                        RCLCPP_DEBUG_STREAM(node_->get_logger(), "unable to find any possible paths, not computing");
                        return nullptr;
                    }

                    // visualize the possible paths
                    this->visualizePaths(possiblePaths);

                    nav_msgs::msg::Path::SharedPtr path = this->findBestPath(possiblePaths);

                    return path;
                }

                void visualizePaths(const std::vector<nav_msgs::msg::Path::SharedPtr> possiblePaths)
                {
                    visualization_msgs::msg::MarkerArray markerArray;
                    int id = 0;
                    for (auto path : possiblePaths)
                    {
                        
                        for (size_t i = 0; i < path->poses.size() - 1; i++)
                        {
                            visualization_msgs::msg::Marker marker;
                            marker.header = path->header;
                            marker.header.stamp = node_->now();
                            marker.ns = "frenet_frame_debug_vis";
                            marker.id = id++;
                            marker.type = visualization_msgs::msg::Marker::SPHERE;
                            marker.action = visualization_msgs::msg::Marker::ADD;
                            marker.pose = path->poses[i].pose;
                            marker.scale.x = 0.1;
                            marker.scale.y = 0.1;
                            marker.scale.z = 0.1;
                            marker.color.a = 1.0;
                            marker.color.r = 0.0;
                            marker.color.g = 1.0;
                            marker.color.b = 1.0;
                            markerArray.markers.push_back(marker);
                        }
                    }
                    this->debug_vis_publisher_->publish(markerArray);
                }

                    nav_msgs::msg::Path::SharedPtr findBestPath(const std::vector<nav_msgs::msg::Path::SharedPtr> possiblePaths)
                {
                    // find the best path
                    double min_cost = std::numeric_limits<double>::max();
                    nav_msgs::msg::Path::SharedPtr best_path;
                    for (auto path : possiblePaths)
                    {
                        double cost = this->computePathCost(path);
                        if (cost < min_cost)
                        {
                            min_cost = cost;
                            best_path = path;
                        }
                    }
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "best path cost: " << min_cost);
                    return best_path;
                }

                double computePathCost(nav_msgs::msg::Path::SharedPtr path) {
                    // compute the cost of the path
                    double cost = 0;
                    for (size_t i = 0; i < path->poses.size() - 1; i++)
                    {
                        double dx = path->poses[i + 1].pose.position.x - path->poses[i].pose.position.x;
                        double dy = path->poses[i + 1].pose.position.y - path->poses[i].pose.position.y;
                        cost += std::sqrt(dx * dx + dy * dy);
                    }
                    return cost;
                }

                std::vector<nav_msgs::msg::Path::SharedPtr> generatePossiblePaths(){
                    RCLCPP_DEBUG_STREAM(node_->get_logger(), "generating possible paths");
                    geometry_msgs::msg::PoseStamped::SharedPtr nextWaypoint = findNextWaypoint(10.0, *m_state_->latest_odom, *m_state_->global_plan_);
                    std::vector<geometry_msgs::msg::PoseStamped> possibleEndpoints;

                    for (double i = -2; i <= 2; i += 1) {
                        for (double j = -2; j <= 2; j += 1)
                        {
                            geometry_msgs::msg::PoseStamped pose;
                            pose.header = m_state_->latest_odom->header;
                            pose.header.stamp = node_->now();
                            pose.pose.position.x = nextWaypoint->pose.position.x + i;
                            pose.pose.position.y = nextWaypoint->pose.position.y + j;
                            pose.pose.position.z = nextWaypoint->pose.position.z;
                            possibleEndpoints.push_back(pose);
                        }
                    }


                    // for every end point, find paths to it
                    std::vector<nav_msgs::msg::Path::SharedPtr> possiblePaths;
                    for (auto endpoint : possibleEndpoints)
                    {
                        nav_msgs::msg::Path::SharedPtr path = std::make_shared<nav_msgs::msg::Path>();
                        path->header = m_state_->global_plan_->header;
                        path->header.stamp = node_->now();

                        // generate n waypoints from odom to next_waypoint
                        int n = 10;
                        double dx = (endpoint.pose.position.x - m_state_->latest_odom->pose.pose.position.x) / n;
                        double dy = (endpoint.pose.position.y - m_state_->latest_odom->pose.pose.position.y) / n;
                        double dz = (endpoint.pose.position.z - m_state_->latest_odom->pose.pose.position.z) / n;

                        for (int i = 0; i < n; i++)
                        {
                            geometry_msgs::msg::PoseStamped pose;
                            pose.header = m_state_->latest_odom->header;
                            pose.header.stamp = node_->now();
                            pose.pose.position.x = m_state_->latest_odom->pose.pose.position.x + dx * i;
                            pose.pose.position.y = m_state_->latest_odom->pose.pose.position.y + dy * i;
                            pose.pose.position.z = m_state_->latest_odom->pose.pose.position.z + dz * i;
                            path->poses.push_back(pose);
                        }
                        possiblePaths.push_back(path);
                    }
                    return possiblePaths;
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
                std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> debug_vis_publisher_;
            };
        } // namespace local
    }     // namespace planning
} // namespace roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::planning::local::SimpleFrenetFramePlanner, roar::planning::local::LocalPlannerPlugin)

#endif // ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_