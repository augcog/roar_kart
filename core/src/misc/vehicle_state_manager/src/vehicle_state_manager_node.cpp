
#include "vehicle_state_manager/vehicle_state_manager_node.hpp"

using namespace roar;

VehicleStateManagerNode::VehicleStateManagerNode() : nav2_util::LifecycleNode("vehicle_state_manager")
{
    this->declare_parameter("frame_id", "ego_vehicle");
    this->declare_parameter("loop_rate", 20.0); // 20 hz

    vehicle_state = std::make_shared<roar_msgs::msg::VehicleState>();
}

VehicleStateManagerNode::~VehicleStateManagerNode()
{
}

nav2_util::CallbackReturn VehicleStateManagerNode::on_configure(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "vehicle_state_manager_node is now configuring.");
    double loop_rate = this->get_parameter("loop_rate").as_double();
    int loop_duration_millis = (int)(1000.0 / loop_rate);
    update_timer = create_wall_timer(std::chrono::milliseconds(loop_duration_millis), std::bind(&VehicleStateManagerNode::update_callback, this));
    vehicle_state_pub_ = this->create_publisher<roar_msgs::msg::VehicleState>(
        "/roar/vehicle_state", 10);

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/roar/odometry", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::odom_callback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/roar/imu", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::imu_callback, this, std::placeholders::_1));

    vehicle_status_sub_ = this->create_subscription<roar_msgs::msg::VehicleStatus>(
        "/roar/vehicle/status", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::vehicle_status_callback, this, std::placeholders::_1));

    //vehicle_state_sub_ = this->create_subscription<roar_msgs::msg::VehicleState>(
        //"/roar/vehicle/state", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::vehicle_state_callback, this, std::placeholders::_1));

    vehicle_control_sub_ = this->create_subscription<roar_msgs::msg::VehicleControl>(
        "/roar/vehicle_control", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::vehicle_control_callback, this, std::placeholders::_1));

    next_waypoint_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/roar/next_waypoint", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::next_waypoint_callback, this, std::placeholders::_1));

    behavior_status_sub_ = this->create_subscription<roar_msgs::msg::BehaviorStatus>(
        "/roar/behavior/status", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::behavior_status_callback, this, std::placeholders::_1));

    global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/roar/global_path", rclcpp::SystemDefaultsQoS(), std::bind(&VehicleStateManagerNode::global_path_callback, this, std::placeholders::_1));

    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VehicleStateManagerNode::on_activate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "vehicle_state_manager_node is now active.");
    vehicle_state_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
}
nav2_util::CallbackReturn VehicleStateManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "vehicle_state_manager_node is now inactive.");
    vehicle_state_pub_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VehicleStateManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "vehicle_state_manager_node is now cleaning up.");
    return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn VehicleStateManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
{
    RCLCPP_INFO(get_logger(), "vehicle_state_manager_node is now shutting down.");
    return nav2_util::CallbackReturn::SUCCESS;
}

void VehicleStateManagerNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    vehicle_state->odometry = *msg;
}

void VehicleStateManagerNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    vehicle_state->imu = *msg;
}

void VehicleStateManagerNode::vehicle_status_callback(const roar_msgs::msg::VehicleStatus::SharedPtr msg)
{
    vehicle_state->vehicle_status = *msg;
}


void VehicleStateManagerNode::vehicle_control_callback(const roar_msgs::msg::VehicleControl::SharedPtr msg)
{
    vehicle_state->vehicle_control = *msg;
}

void VehicleStateManagerNode::next_waypoint_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    vehicle_state->next_waypoint = *msg;
}

void VehicleStateManagerNode::behavior_status_callback(const roar_msgs::msg::BehaviorStatus::SharedPtr msg)
{
    vehicle_state->behavior_status = *msg;
}

void VehicleStateManagerNode::global_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
    vehicle_state->global_path = *msg;
}

void VehicleStateManagerNode::update_callback()
{
    // update vehicle state
    vehicle_state->header.stamp = this->now();
    vehicle_state->header.frame_id = this->get_parameter("frame_id").as_string();

    // publish vehicle state
    vehicle_state_pub_->publish(*vehicle_state);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<roar::VehicleStateManagerNode>();

    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}