#include "global_planning/global_planner_interface.hpp"
#include "global_planning/planners/race_planner.hpp"
#include <sstream>
#include <fstream>
#include <nav_msgs/msg/odometry.hpp>
namespace ROAR
{
    namespace GlobalPlanning
    {
        RacePlanner::RacePlanner(nav2_util::LifecycleNode *node) : GlobalPlannerInterface(node, "RacePlanner")
        {
            this->m_node_->declare_parameter("RacePlanner.waypoint_path", "waypoints.txt");
            this->m_node_->declare_parameter("map_frame", "map");
            RCLCPP_INFO_STREAM(m_logger_, "Waypoint_path: " << this->m_node_->get_parameter("RacePlanner.waypoint_path").as_string());
        }
        RacePlanner::~RacePlanner()
        {
        }

        void RacePlanner::initialize()
        {
            // read in waypoints from waypoints.txt
            this->read_waypoints(this->m_node_->get_parameter("RacePlanner.waypoint_path").as_string());
        }

        StepResult RacePlanner::step(const StepInput input)
        {
            RCLCPP_DEBUG_STREAM(m_logger_, "-----");

            StepResult result;
            nav_msgs::msg::Odometry::SharedPtr odom = input.odom;

            // construct global path, if exist
            if (global_path_msg != nullptr)
            {
                RCLCPP_DEBUG_STREAM(m_logger_, "Global path: [" << global_path_msg->poses.size() << "]");
                result.global_path = global_path_msg;
            } else {
                RCLCPP_DEBUG_STREAM(m_logger_, "Global path: [NULL]");
            }

            return result;
        }

        void RacePlanner::read_waypoints(const std::string &file_path)
        {
            std::ifstream infile(file_path);

            if (!infile.is_open())
            {
                RCLCPP_ERROR(m_logger_, "Failed to open waypoints file: %s", file_path.c_str());
                return;
            }

            // Skip the first line with column names
            std::string line;
            std::getline(infile, line);

            while (std::getline(infile, line))
            {
                std::istringstream iss(line);
                std::string timestamp_str, frame_id_str, child_frame_id_str,
                    x_str, y_str, z_str,
                    qx_str, qy_str, qz_str, qw_str,
                    vx_str, vy_str, vz_str,
                    wx_str, wy_str, wz_str;

                // Extract each value from the comma-separated line
                if (!(std::getline(iss, timestamp_str, ',') &&
                      std::getline(iss, frame_id_str, ',') &&
                      std::getline(iss, child_frame_id_str, ',') &&
                      std::getline(iss, x_str, ',') &&
                      std::getline(iss, y_str, ',') &&
                      std::getline(iss, z_str, ',') &&
                      std::getline(iss, qx_str, ',') &&
                      std::getline(iss, qy_str, ',') &&
                      std::getline(iss, qz_str, ',') &&
                      std::getline(iss, qw_str, ',') &&
                      std::getline(iss, vx_str, ',') &&
                      std::getline(iss, vy_str, ',') &&
                      std::getline(iss, vz_str, ',') &&
                      std::getline(iss, wx_str, ',') &&
                      std::getline(iss, wy_str, ',') &&
                      std::getline(iss, wz_str, ',')))
                {
                    RCLCPP_WARN(m_logger_, "Invalid waypoint format: %s", line.c_str());
                    continue;
                }

                try
                {
                    // Convert the string values to floats
                    double x = std::stod(x_str);
                    double y = std::stod(y_str);
                    double z = std::stod(z_str);
                    double qx = std::stod(qx_str);
                    double qy = std::stod(qy_str);
                    double qz = std::stod(qz_str);
                    double qw = std::stod(qw_str);
                    double vx = std::stod(vx_str);
                    double vy = std::stod(vy_str);
                    double vz = std::stod(vz_str);
                    double wx = std::stod(wx_str);
                    double wy = std::stod(wy_str);
                    double wz = std::stod(wz_str);

                    nav_msgs::msg::Odometry waypoint;
                    waypoint.header.stamp = rclcpp::Time(std::stod(timestamp_str));
                    waypoint.header.frame_id = frame_id_str;
                    waypoint.child_frame_id = child_frame_id_str;
                    waypoint.pose.pose.position.x = x;
                    waypoint.pose.pose.position.y = y;
                    waypoint.pose.pose.position.z = z;
                    waypoint.pose.pose.orientation.x = qx;
                    waypoint.pose.pose.orientation.y = qy;
                    waypoint.pose.pose.orientation.z = qz;
                    waypoint.pose.pose.orientation.w = qw;
                    waypoint.twist.twist.linear.x = vx;
                    waypoint.twist.twist.linear.y = vy;
                    waypoint.twist.twist.linear.z = vz;
                    waypoint.twist.twist.angular.x = wx;
                    waypoint.twist.twist.angular.y = wy;
                    waypoint.twist.twist.angular.z = wz;

                    waypoints_.push_back(waypoint);
                }
                catch (const std::exception &ex)
                {
                    RCLCPP_ERROR(m_logger_, "Error parsing waypoint: %s", ex.what());
                }
            }
            RCLCPP_INFO(m_logger_, "Loaded %lu waypoints", waypoints_.size());

            // fill in global_path_msg if waypoints_ is not empty
            if (waypoints_.size() > 0)
            {
                global_path_msg = std::make_shared<nav_msgs::msg::Path>();
                global_path_msg->header.frame_id = this->m_node_->get_parameter("map_frame").as_string();
                global_path_msg->header.stamp = m_node_->now();
                global_path_msg->poses.resize(waypoints_.size());
                for (size_t i = 0; i < waypoints_.size(); i++)
                {
                    global_path_msg->poses[i].header = waypoints_[i].header;
                    global_path_msg->poses[i].pose = waypoints_[i].pose.pose;
                }
            }
            infile.close();
        }
    }
}