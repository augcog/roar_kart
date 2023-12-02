#include "controller_manager/controller_manager_ros.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<controller::ControllerManagerNode>();
  
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}