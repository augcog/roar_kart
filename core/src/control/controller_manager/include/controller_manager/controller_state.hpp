#ifndef ROAR_CONTROL__PLUGIN__STATE_HPP_
#define ROAR_CONTROL__PLUGIN__STATE_HPP_
#include "nav_msgs/msg/path.hpp"
#include "roar_msgs/msg/behavior_status.hpp"
#include "roar_msgs/msg/vehicle_state.hpp"

namespace roar
{
    namespace control
    {
        struct ControllerManagerState
        {
            typedef std::shared_ptr<ControllerManagerState> SharedPtr;

            nav_msgs::msg::Path path_ego_centric;
            roar_msgs::msg::BehaviorStatus::SharedPtr behavior_status;
            roar_msgs::msg::VehicleState::SharedPtr vehicle_state;
        };

        struct ControllerManagerConfig
        {
            typedef std::shared_ptr<ControllerManagerConfig> SharedPtr;
            typedef std::unique_ptr<ControllerManagerConfig> UniquePtr;

            bool debug;
            double loop_rate_millis;

            double target_speed;
            double max_speed;
            std::string base_link_frame;
            std::string map_frame;

            double min_distance;
        };
    } // namespace control
} // namespace roar

#endif // ROAR_CONTROL__PLUGIN__STATE_HPP_