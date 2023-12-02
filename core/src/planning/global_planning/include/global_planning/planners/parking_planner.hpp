#include "global_planning/global_planner_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include "planning_interfaces/srv/load_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <tf2_ros/transform_listener.h>

namespace ROAR
{
    struct NavPlannerGlobalPathFinderInputs
    {
        nav_msgs::msg::Odometry::SharedPtr odom;
        geometry_msgs::msg::PoseStamped::SharedPtr goal_pose;
        nav_msgs::msg::OccupancyGrid::SharedPtr global_map;
    };

    struct NavPlannerGlobalPathFinderOutput
    {
        bool status;
        nav_msgs::msg::Path::SharedPtr global_path;
    };

    struct NavPlannerNextWaypointFinderInputs
    {
        nav_msgs::msg::Path::SharedPtr global_path;
        geometry_msgs::msg::PoseStamped::SharedPtr current_pose;
    };

    struct NavPlannerNextWaypointFinderOutputs
    {
        bool status;
        geometry_msgs::msg::PoseStamped::SharedPtr target_waypoint;
    };

    namespace GlobalPlanning
    {
        class ParkingPlanner : public GlobalPlannerInterface
        {
        public:
            ParkingPlanner(nav2_util::LifecycleNode *node);
            ~ParkingPlanner();
            void initialize();
            StepResult step(const StepInput input);

        protected:
            bool checkGlobalMap()
            {
                return m_global_map == nullptr ? false : true;
            }
            bool checkGoalWithinGlobalMap();
            bool on_configure();
            bool on_activate();
            bool checkVehicleStatus(const StepInput &input)
            {
                return input.odom == nullptr ? false : true;
            }

            NavPlannerGlobalPathFinderOutput planTrajectory(const NavPlannerGlobalPathFinderInputs &inputs);

            bool didReceiveGoalPose()
            {
                return m_goal_pose_stamped == nullptr ? false : true;
            }

            // declare subscriber for goal pose
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_goal_pose_subscriber;
            void onReceiveGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr m_global_map_subscriber;
            void onReceiveGlobalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
            {
                RCLCPP_INFO(m_logger_, "Received global map");
                m_global_map = msg;
            }

        private:
            float path_step = 1.0;
            nav_msgs::msg::Path::SharedPtr m_global_path;
            geometry_msgs::msg::PoseStamped::SharedPtr m_next_waypoint_pose_stamped;

            geometry_msgs::msg::PoseStamped::SharedPtr m_goal_pose_stamped;
            bool didGoalPoseUpdated = false;

            nav_msgs::msg::OccupancyGrid::SharedPtr m_global_map;

            nav_msgs::msg::Odometry::SharedPtr m_odom;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            void p_debugGlobalTrajectoryInputs(const NavPlannerGlobalPathFinderInputs &inputs)
            {
                // print global map info
                RCLCPP_DEBUG_STREAM(m_logger_, "\nGlobal Map Info:"
                                                   << "\n   frame: " << inputs.global_map->header.frame_id
                                                   << "\n   width: " << m_global_map->info.width
                                                   << "\n   height: " << m_global_map->info.height
                                                   << "\n   resolution: " << m_global_map->info.resolution
                                                   << "\n   origin: " << m_global_map->info.origin.position.x << ", " << m_global_map->info.origin.position.y);
                // print vehicle odom info
                RCLCPP_DEBUG_STREAM(m_logger_, "\nVehicle Odom:"
                                                   << "\n   frame: " << inputs.odom->header.frame_id
                                                   << "\n   position: " << inputs.odom->pose.pose.position.x << ", " << inputs.odom->pose.pose.position.y
                                                   << "\n   orientation: " << inputs.odom->pose.pose.orientation.x << ", " << inputs.odom->pose.pose.orientation.y << ", " << inputs.odom->pose.pose.orientation.z << ", " << inputs.odom->pose.pose.orientation.w);

                // print goal pose info
                RCLCPP_DEBUG_STREAM(m_logger_, "\nGoal Pose:"
                                                   << "\n   frame: " << inputs.goal_pose->header.frame_id
                                                   << "\n   position: " << m_goal_pose_stamped->pose.position.x << ", " << m_goal_pose_stamped->pose.position.y
                                                   << "\n   orientation: " << m_goal_pose_stamped->pose.orientation.x << ", " << m_goal_pose_stamped->pose.orientation.y << ", " << m_goal_pose_stamped->pose.orientation.z << ", " << m_goal_pose_stamped->pose.orientation.w);
            }
            void p_debugPath(nav_msgs::msg::Path::SharedPtr global_path)
            {
                if (global_path == nullptr)
                {
                    RCLCPP_DEBUG(m_logger_, "Global Path is null");
                    return;
                }
                RCLCPP_DEBUG_STREAM(m_logger_, "Global Path: frame_id:" << global_path->header.frame_id << " | Number of waypoints: " << global_path->poses.size());
            }

            float p_safe_cast_to_map(float input, float map_origin, float map_resolution, float map_max)
            {
                float output = (input - map_origin) / map_resolution;
                
                if (output < 0)
                {
                    output = 0;
                }
                else if (output >= map_max)
                {
                    output = map_max-1;
                }
                return output;
            }
        };
    }
} // namespace ROAR
