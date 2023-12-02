#ifndef ROAR_PLANNING__PLUGIN__STATE_HPP_
#define ROAR_PLANNING__PLUGIN__STATE_HPP_
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace roar 
{
    namespace planning
    {
        namespace local
        {
            struct LocalPlannerManagerState
            {
                typedef std::shared_ptr<LocalPlannerManagerState> SharedPtr;

                std::shared_ptr<geometry_msgs::msg::PolygonStamped> robot_footprint;
                std::shared_ptr<nav_msgs::msg::Odometry> odom;
                std::shared_ptr<nav_msgs::msg::Path> global_plan;
                std::shared_ptr<nav_msgs::msg::OccupancyGrid> occupancy_map;
            };
            
            struct LocalPlannerManagerConfig
            {
                typedef std::shared_ptr<LocalPlannerManagerConfig> SharedPtr;

                bool debug;
                double loop_rate;
            };
        } // namespace local
    } // namespace planning
} // namespace roar

#endif // ROAR_PLANNING__PLUGIN__STATE_HPP_