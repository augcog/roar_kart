#include "costmap_node_manager/costmap_node_manager.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace costmap_node_manager
{
  CostmapNodeManager::CostmapNodeManager()
      : LifecycleNode("costmap_node_manager", "", true)
  {
    RCLCPP_INFO(get_logger(), "Creating Costmap Manager Node");
  }

  CostmapNodeManager::~CostmapNodeManager()
  {
    RCLCPP_INFO(get_logger(), "Destroying ROS2Costmap2DNode");
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring");
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "local_costmap", std::string{get_namespace()}, "local_costmap");
    costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);
    costmap_ros_->configure();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    costmap_ros_->activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    costmap_ros_->deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    costmap_ros_->cleanup();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Shutting Down");
    costmap_ros_->shutdown();
    return nav2_util::CallbackReturn::SUCCESS;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<costmap_node_manager::CostmapNodeManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}