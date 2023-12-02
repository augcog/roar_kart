#include <cstdio>
#include "behavior_planning/behavior_planner_manager.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cmath> // For sqrt and atan2 functions

namespace ROAR
{
    namespace BehaviorPlanning
    {
        BehaviorPlannerManager::BehaviorPlannerManager() : nav2_util::LifecycleNode("global_planner_manager")
        {
        }

        BehaviorPlannerManager::~BehaviorPlannerManager()
        {
            // Destructor logic, if any
            RCLCPP_DEBUG(get_logger(), "BehaviorPlannerManager is destroyed.");
        }
        nav2_util::CallbackReturn BehaviorPlannerManager::on_configure(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "BehaviorPlannerManager is now configured.");

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn BehaviorPlannerManager::on_activate(const rclcpp_lifecycle::State &state)
        {
            behavior_status_pub_
            RCLCPP_DEBUG(get_logger(), "BehaviorPlannerManager is now active.");
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn BehaviorPlannerManager::on_deactivate(const rclcpp_lifecycle::State &state)
        {

            RCLCPP_DEBUG(get_logger(), "BehaviorPlannerManager is now inactive.");
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn BehaviorPlannerManager::on_cleanup(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "BehaviorPlannerManager is now cleaned up.");
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn BehaviorPlannerManager::on_shutdown(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "BehaviorPlannerManager is now shutting down.");
            // Custom shutdown logic goes here
            return nav2_util::CallbackReturn::SUCCESS;
        }

    } // namespace global_planning
} // namespace roar

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROAR::BehaviorPlanning::BehaviorPlannerManager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
