#include <cstdio>
#include "global_planning/global_planner_manager.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cmath> // For sqrt and atan2 functions
#include "global_planning/global_planner_interface.hpp"
#include "global_planning/planners/race_planner.hpp"
#include "global_planning/planners/parking_planner.hpp"

namespace ROAR
{
    namespace GlobalPlanning
    {
        GlobalPlannerManager::GlobalPlannerManager() : nav2_util::LifecycleNode("global_planner_manager")
        {
            this->declare_parameter("debug", false);
            this->declare_parameter("planner_name", "RacePlanner");
            this->declare_parameter("planner_frequency", 0.1);

            std::string planner_name = this->get_parameter("planner_name").as_string();

            if (this->get_parameter("debug").as_bool())
            {
                auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
                auto _ = rcutils_logging_set_logger_level(planner_name.c_str(), RCUTILS_LOG_SEVERITY_DEBUG);      // enable or disable debug
                _ = rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_DEBUG);
            }

            if (planner_name == "RacePlanner")
            {
                this->planner = new RacePlanner(this);
            }
            else if (planner_name == "ParkingPlanner")
            {
                this->planner = new ParkingPlanner(this);
            }

            RCLCPP_INFO_STREAM(get_logger(), "Global Planner Manager has been initialized. Debug: " << this->get_parameter("debug").as_bool() << " Using Planner: " << planner_name);
        }

        GlobalPlannerManager::~GlobalPlannerManager()
        {
            // Destructor logic, if any
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is destroyed.");
        }
        nav2_util::CallbackReturn GlobalPlannerManager::on_configure(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now configured.");

            this->planner->initialize();

            // subscribers
            current_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/roar/odometry",
                rclcpp::QoS(rclcpp::KeepLast(1)),
                [&](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    this->current_odom = msg;
                });

            // publishers
            this->global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
            if (!this->planner->on_configure())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }
            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_activate(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now activating.");
            this->timer = this->create_wall_timer(
                std::chrono::milliseconds((int)(this->get_parameter("planner_frequency").as_double() * 1000)),
                std::bind(&GlobalPlannerManager::step, this));

            this->global_path_publisher_->on_activate();

            if (!this->planner->on_activate())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_deactivate(const rclcpp_lifecycle::State &state)
        {
            this->global_path_publisher_->on_deactivate();
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now inactive.");
            if (!this->planner->on_deactivate())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_cleanup(const rclcpp_lifecycle::State &state)
        {

            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now cleaned up.");
            if (!this->planner->on_cleanup())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_shutdown(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now shutting down.");
            if (!this->planner->on_shutdown())
            {
                RCLCPP_ERROR(get_logger(), "Failed to shutdown planner");
                return nav2_util::CallbackReturn::FAILURE;
            }

            return nav2_util::CallbackReturn::SUCCESS;
        }

        void GlobalPlannerManager::step()
        {
            if (this->planner == nullptr)
            {
                RCLCPP_ERROR(get_logger(), "Planner is not initialized");
                return;
            }
            if (this->current_odom == nullptr)
            {
                // probably not initialized yet
                // RCLCPP_ERROR(get_logger(), "Current odom is not initialized");
                return;
            }
            // RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now stepping.");
            StepInput input;
            input.odom = this->current_odom;
            StepResult result = this->planner->step(input);

            // publish global path
            if (result.global_path != nullptr)
            {
                // RCLCPP_DEBUG(get_logger(), "Publishing global path");
                this->global_path_publisher_->publish(*result.global_path);
            }
            else
            {
                // RCLCPP_DEBUG(get_logger(), "Global path is null, try selecting a path?");
            }
        }

    } // namespace global_planning
} // namespace roar

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROAR::GlobalPlanning::GlobalPlannerManager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
