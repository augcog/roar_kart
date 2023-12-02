#ifndef ROAR__BEHAVIOR_PLANNING__UTILS_HPP_
#define ROAR__BEHAVIOR_PLANNING__UTILS_HPP_
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cmath>
#include "roar_msgs/msg/behavior_status.hpp"
#include "roar_msgs/msg/vehicle_state.hpp"
namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            struct BTInputs
            {
                typedef std::shared_ptr<BTInputs> SharedPtr;
                typedef std::shared_ptr<const BTInputs> ConstSharedPtr;
                typedef std::unique_ptr<BTInputs> UniquePtr;

                roar_msgs::msg::VehicleState::SharedPtr vehicle_state{};
            };

            struct BTOutputs
            {
                typedef std::shared_ptr<BTOutputs> SharedPtr;
                typedef std::shared_ptr<const BTOutputs> ConstSharedPtr;
                typedef std::unique_ptr<BTOutputs> UniquePtr;

                roar_msgs::msg::BehaviorStatus behavior_status;
            };

            namespace utils
            {

            } // namespace utils
        }     // namespace behavior_planning
    }         // namespace planning
} // namespace roar

#endif // ROAR__BEHAVIOR_PLANNING__UTILS_HPP_