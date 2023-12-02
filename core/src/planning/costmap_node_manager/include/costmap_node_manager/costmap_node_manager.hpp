#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#ifndef COSTMAP_NODE_MANAGER_HPP_
#define COSTMAP_NODE_MANAGER_HPP_

namespace costmap_node_manager
{
    class CostmapNodeManager : public nav2_util::LifecycleNode
    {
    public:
        /**
         * @brief Constructor for roar_ros2_costmap_2d::ROS2Costmap2DNode
         */
        CostmapNodeManager();
        /**
         * @brief Destructor for roar_ros2_costmap_2d::ROS2Costmap2DNode
         */
        ~CostmapNodeManager();

    protected:
        /**
         * @brief Configures controller parameters and member variables
         *
         * Configures costmap;
         * @param state LifeCycle Node's state
         * @return Success or Failure
         * @throw pluginlib::PluginlibException When failed to initialize controller
         * plugin
         */
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Activates member variables
         *
         * Activates costmap
         * server
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Deactivates member variables
         *
         * Deactivates costmap
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Calls clean up states and resets member variables.
         *
         * Costmap clean up state is called, and resets rest of the
         * variables
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Called when in Shutdown state
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        /**
         * Start of variable section
        */
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> costmap_thread_;
    };
}
#endif // COSTMAP_NODE_MANAGER_HPP_
