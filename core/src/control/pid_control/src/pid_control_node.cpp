#include "pid_control/pid_control_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/header.hpp>
#include <iostream>
#include <fstream>
#include <iomanip> // for std::setprecision
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pid_controller
{
    PIDControlActionServer::PIDControlActionServer(const rclcpp::NodeOptions &options) : LifecycleNode("pid_control_action_server", "",true)
    {
        RCLCPP_INFO(get_logger(), "Creating PID Control Server Node");
        longitudinal_pid_configs = new std::map<float, control_toolbox::Pid::Gains>;
        lateral_pid_configs = new std::map<float, control_toolbox::Pid::Gains>;

        this->declare_parameter("debug", false);
        this->declare_parameter("loop_rate", 10.0);
        this->declare_parameter("pid_config_file_path", "pid_config.json");
        this->declare_parameter("goal_threshold", 2.0);
        this->declare_parameter("max_right_steering_angle_rad", 1.0);
        this->declare_parameter("max_left_steering_angle_rad", -1.0);

        this->MAX_RIGHT_STEERING_ANGLE = this->get_parameter("max_right_steering_angle_rad").as_double();
        this->MAX_LEFT_STEERING_ANGLE = this->get_parameter("max_left_steering_angle_rad").as_double();
        pid_last_calc_timestamp = this->get_clock()->now();
    }
    
    PIDControlActionServer::~PIDControlActionServer()
    {
        RCLCPP_INFO(this->get_logger(), "Destroying PIDControlActionServer");
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.drive.jerk = 0;
        this->ackermann_publisher_->publish(msg);
    }
    
    void PIDControlActionServer::execute(const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        this->thread_mutex = true;
        rclcpp::WallRate loop_rate(this->get_parameter("loop_rate").as_double());
        auto result = std::make_shared<ControlAction::Result>();

        while (rclcpp::ok())
        {
            if (this->thread_mutex == false)
            {
                // thread is terminated from outside for whatever reason
                result->status = -1;
                goal_handle->succeed(result);
                return;
            }

            if (goal_handle->is_canceling())
            {
                result->status = 0; // cancelled
                goal_handle->canceled(result);
                RCLCPP_INFO(get_logger(), "goal canceled");
                return;
            }

            if (this->hasReachedGoalThreshold(goal_handle))
            {
                // close enough to the end waypoint
                RCLCPP_DEBUG(get_logger(), "Goal completed");
                result->status = 0;
                goal_handle->succeed(result);
                return;
            }

            if (this->latest_odom == nullptr)
            {
                loop_rate.sleep();
                continue;
            }
            rclcpp::Time end_time = this->get_clock()->now();
            rclcpp::Duration dt = end_time - pid_last_calc_timestamp;

            this->pFollowWaypoint(this->get_clock()->now() - pid_last_calc_timestamp, goal_handle);
            loop_rate.sleep();
            pid_last_calc_timestamp = end_time;
        }

        this->thread_mutex = false;
    }

    nav2_util::CallbackReturn
    PIDControlActionServer::on_configure(const rclcpp_lifecycle::State &state)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/carla/ego_vehicle/odometry", rclcpp::SystemDefaultsQoS(),
            std::bind(&PIDControlActionServer::onLatestOdomReceived, this, std::placeholders::_1));
        ackermann_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_drive", 10);
        next_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pid_controller/next_waypoint_visualization", 10);

        action_server_ = rclcpp_action::create_server<ControlAction>(
            get_node_base_interface(),
            get_node_clock_interface(),
            get_node_logging_interface(),
            get_node_waitables_interface(),
            "pid_control",
            std::bind(&PIDControlActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PIDControlActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PIDControlActionServer::handle_accepted, this, std::placeholders::_1));
        longitudinal_pid_node_ = rclcpp::Node::make_shared("longitudinal_pid_node");
        lateral_pid_node_ = rclcpp::Node::make_shared("lateral_pid_node");

        longitudinal_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->longitudinal_pid_node_, "longitudinal_pid_controller"));
        lateral_pid = std::shared_ptr<control_toolbox::PidROS>(new control_toolbox::PidROS(this->lateral_pid_node_, "lateral_pid_controller"));

        bool pid_read_status = this->pReadPidFromFile();
        if (!pid_read_status)
        {
            RCLCPP_ERROR(this->get_logger(), "Error reading pid configuration");
            return nav2_util::CallbackReturn::FAILURE;
        }

        this->pUpdateLongAndLatPID(0.0);
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "PID Control Server Configured");

        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn PIDControlActionServer::on_activate(const rclcpp_lifecycle::State &state)
    {

        this->ackermann_publisher_->on_activate();
        this->next_waypoint_pub_->on_activate();
        RCLCPP_INFO(this->get_logger(), "PID Control Server Activated");

        return nav2_util::CallbackReturn::SUCCESS;
    }
    
    nav2_util::CallbackReturn PIDControlActionServer::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "PID Control Server Deactivated");

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn PIDControlActionServer::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(this->get_logger(), "PID Control Server Shutdown");

        return nav2_util::CallbackReturn::SUCCESS;
    }

    rclcpp_action::GoalResponse PIDControlActionServer::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ControlAction::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with [%d] waypoints", goal->path.poses.size());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse PIDControlActionServer::handle_cancel(
        const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void PIDControlActionServer::handle_accepted(const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&PIDControlActionServer::execute, this, _1), goal_handle}.detach();
    }
   
    void PIDControlActionServer::onLatestOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        this->latest_odom = msg;
    }


    void PIDControlActionServer::pFollowWaypoint(rclcpp::Duration dt, const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        auto feedback = std::make_shared<ControlAction::Feedback>();

        // find the immediate next point to follow
        float target_spd = goal_handle->get_goal().get()->target_spd;
        std::shared_ptr<nav_msgs::msg::Odometry> odom = this->latest_odom;
        nav_msgs::msg::Path path = goal_handle->get_goal().get()->path;
        int next_waypoint_index = this->findNextWaypoint(path, odom);

        geometry_msgs::msg::Pose::SharedPtr next_waypoint_ego_fov = this->pConvertToEgoFov(path.poses[next_waypoint_index].pose);
        if (next_waypoint_ego_fov == nullptr){
            return;
        }

        if (true)
        // if (this->get_parameter("debug").as_bool())
        {
            this->pPublishVisualizationFromPose(next_waypoint_ego_fov);
        }

        // Run PID controller
        float throttle_error = this->calculateThrottleError(odom, target_spd);
        float steering_error = this->calculateSteeringError(odom, next_waypoint_ego_fov);

        float current_spd = this->pSpeedFromOdom(odom);
        this->pUpdateLongAndLatPID(current_spd);
        float throttle_cmd = std::max(0.0f, std::min(1.0f, float(this->longitudinal_pid->computeCommand(throttle_error, dt))));
        float steering_cmd = std::max(MAX_LEFT_STEERING_ANGLE, 
                                      std::min(MAX_RIGHT_STEERING_ANGLE, 
                                      float(this->lateral_pid->computeCommand(steering_error, dt))));

        this->pPublishAckermannMsg(steering_error,
                                   steering_cmd,
                                   throttle_cmd,
                                   odom->header.frame_id);

        feedback->curr_index = next_waypoint_index;
        goal_handle->publish_feedback(feedback);
    }

    geometry_msgs::msg::Pose::SharedPtr PIDControlActionServer::pConvertToEgoFov(geometry_msgs::msg::Pose original)
    {
        geometry_msgs::msg::TransformStamped t;
        std::string fromFrameRel = this->latest_odom->header.frame_id;
        std::string toFrameRel = this->latest_odom->child_frame_id;
        try
        {
          t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
          RCLCPP_INFO(
              this->get_logger(), "Could not transform %s to %s: %s",
              toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          return nullptr;
        }
        std_msgs::msg::Header header;
        header.frame_id = toFrameRel;
        header.stamp = this->get_clock()->now();
        geometry_msgs::msg::PoseStamped child_pose;
        geometry_msgs::msg::PoseStamped ps;
        ps.header = header;
        ps.pose = original;
        tf2::doTransform(ps, child_pose, t);
        return std::make_shared<geometry_msgs::msg::Pose>(child_pose.pose);
    }

    bool PIDControlActionServer::hasReachedGoalThreshold(const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        if (!latest_odom)
        {
        return false;
        }

        nav_msgs::msg::Path path = goal_handle->get_goal()->path;
        if (path.poses.empty())
        {
        return false;
        }

        geometry_msgs::msg::PoseStamped goal_pose = path.poses.back();

        double distance = sqrt(pow(goal_pose.pose.position.x - latest_odom->pose.pose.position.x, 2) +
                               pow(goal_pose.pose.position.y - latest_odom->pose.pose.position.y, 2));

        // check if we have reached the goal threshold
        if (distance <= this->get_parameter("goal_threshold").as_double())
        {
            return true;
        }

        return false;
    }

    float PIDControlActionServer::pSpeedFromOdom(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        geometry_msgs::msg::Vector3 linear_velocities = odom->twist.twist.linear;
        double vx = linear_velocities.x;
        double vy = linear_velocities.y;
        double vz = linear_velocities.z;
        float current_spd = std::sqrt(vx * vx + vy * vy + vz * vz);
        return current_spd;
    }

    void PIDControlActionServer::pPublishVisualizationFromPose(geometry_msgs::msg::Pose::SharedPtr pose)
    {
        if (this->latest_odom)
        {
            visualization_msgs::msg::Marker marker;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = *pose;
            // marker.pose.position.x = 5.0;
            // marker.pose.position.y = 0.0;
            // marker.pose.position.z = 1.0;
            // Set the marker scale
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            // Set the marker color
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration(std::chrono::seconds(2));

            std_msgs::msg::Header header;

            header.frame_id = this->latest_odom->child_frame_id;
            header.stamp = this->get_clock()->now();

            // Set the header in the marker message
            marker.header = header;
            this->next_waypoint_pub_->publish(marker);
        }
    }
    
    bool
    PIDControlActionServer::pReadPidFromFile()
    {
        std::string pid_config_file_path = this->get_parameter("pid_config_file_path").as_string();
        RCLCPP_INFO(this->get_logger(), "Loading PID configuration from: [%s]", pid_config_file_path.c_str());

        std::ifstream file(pid_config_file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Unable to open file %s", pid_config_file_path.c_str());
            std::cerr << "Failed to open file." << std::endl;
            return false;
        }

        // Parse the JSON data from the file
        rapidjson::IStreamWrapper isw(file);
        rapidjson::Document doc;
        doc.ParseStream(isw);
        if (doc.HasMember("longitudinal_controller"))
        {
            const rapidjson::Value &configs = doc["longitudinal_controller"];
            bool status = this->pLoadConfigs(configs, this->longitudinal_pid_configs);
            RCLCPP_INFO(this->get_logger(), "Longitudinal PIDs:");

            this->pPrettyPrintMap(this->longitudinal_pid_configs);
            if (!status)
            {
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "JSON does not container key [longitudinal_controller]");
            return false;
        }

        if (doc.HasMember("latitudinal_controller"))
        {
            const rapidjson::Value &configs = doc["latitudinal_controller"];
            bool status = this->pLoadConfigs(configs, this->lateral_pid_configs);
            RCLCPP_INFO(this->get_logger(), "Lateral PIDs:");

            this->pPrettyPrintMap(this->lateral_pid_configs);
            if (!status)
            {
                return false;
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "JSON does not container key [latitudinal_controller]");
            return false;
        }
        return true;
    }

    bool
    PIDControlActionServer::pLoadConfigs(const rapidjson::Value &config,
                                         std::map<float, control_toolbox::Pid::Gains> *configs)
    {
        if (!config.IsObject())
        {
            return false;
        }

        for (auto itr = config.MemberBegin(); itr != config.MemberEnd(); ++itr)
        {
            std::string key = itr->name.GetString();
            float f = std::stof(key);

            const rapidjson::Value &pid_config = itr->value;
            (*configs)[f] = this->pConvertJsonToPid(pid_config);
        }

        return true;
    }
    control_toolbox::Pid::Gains
    PIDControlActionServer::pConvertJsonToPid(const rapidjson::Value &config)
    {
        control_toolbox::Pid::Gains gains(0, 0, 0, 0, 0, false);
        if (!config.IsObject())
        {
            return gains;
        }
        if (config.HasMember("k_p") && config["k_p"].IsNumber())
        {
            gains.p_gain_ = config["k_p"].GetFloat();
        }
        if (config.HasMember("k_i") && config["k_i"].IsNumber())
        {
            gains.i_gain_ = config["k_i"].GetFloat();
        }
        if (config.HasMember("k_d") && config["k_d"].IsNumber())
        {
            gains.d_gain_ = config["k_d"].GetFloat();
        }
        if (config.HasMember("i_clamp_min") && config["i_clamp_min"].IsNumber())
        {
            gains.i_min_ = config["i_clamp_min"].GetFloat();
        }
        if (config.HasMember("i_clamp_max") && config["i_clamp_max"].IsNumber())
        {
            gains.i_max_ = config["i_clamp_max"].GetFloat();
        }
        if (config.HasMember("antiwindup") && config["antiwindup"].IsBool())
        {
            gains.antiwindup_ = config["antiwindup"].GetBool();
        }
        return gains;
    }

    void PIDControlActionServer::pPrettyPrintMap(const std::map<float, control_toolbox::Pid::Gains> *configs)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2); // set precision to 2 decimal places
        oss << "\n";
        for (const auto &pair : *configs)
        {
            oss << "  Below " << pair.first << " m/s"
                << ": { p: " << pair.second.p_gain_
                << ", i: " << pair.second.i_gain_
                << ", d: " << pair.second.d_gain_
                << ", i_min_: " << pair.second.i_min_
                << ", i_max_: " << pair.second.i_max_
                << " }\n";
        }
        RCLCPP_INFO(this->get_logger(), oss.str());
    }

    void PIDControlActionServer::pPublishAckermannMsg(float steering_error, float steering_cmd, float throttle_cmd, std::string frame_id)
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.frame_id = frame_id;
        msg.header.stamp = this->get_clock()->now();
        msg.drive.jerk = std::max(-1.0f, std::min(1.0f, throttle_cmd));
        msg.drive.steering_angle = steering_error;
        msg.drive.steering_angle_velocity = std::max(-1.0f, std::min(1.0f, steering_cmd));
        // TODO: add in reverse here
        ackermann_publisher_->publish(msg);
    }
    control_toolbox::Pid::Gains PIDControlActionServer::pGainsFromSpeed(float spd, std::map<float, control_toolbox::Pid::Gains> *configs)
    {
        // Check if map is empty
        if (configs->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty gains configuration, using default gains");
            return control_toolbox::Pid::Gains(DEFAULT_K_P, DEFAULT_K_I, DEFAULT_K_D, 0.0, 0.0, false);
        }

        float closest_speed = configs->begin()->first;
        control_toolbox::Pid::Gains gains = configs->begin()->second;
        for (const auto &pair : *configs)
        {
            if (std::abs(pair.first - spd) < std::abs(closest_speed - spd))
            {
                gains = pair.second;
                closest_speed = pair.first;
            }
        }
        return gains;
    }

    void PIDControlActionServer::pUpdateLongAndLatPID(float spd)
    {
        control_toolbox::Pid::Gains long_gains = this->pGainsFromSpeed(spd, this->longitudinal_pid_configs);
        control_toolbox::Pid::Gains lat_gains = this->pGainsFromSpeed(spd, this->lateral_pid_configs);

        longitudinal_pid->setGains(long_gains);
        lateral_pid->setGains(lat_gains);
    }

    int PIDControlActionServer::findNextWaypoint(const nav_msgs::msg::Path path, const std::shared_ptr<nav_msgs::msg::Odometry> odom)
    {
        // TODO: Better strategy to find the next immediate point to follow 
        return 0;
    }

    float PIDControlActionServer::calculateThrottleError(nav_msgs::msg::Odometry::SharedPtr odom, float target_spd)
    {
        float current_spd = this->pSpeedFromOdom(odom);
        return target_spd - current_spd;
    }
    
    float PIDControlActionServer::calculateSteeringError(nav_msgs::msg::Odometry::SharedPtr odom, geometry_msgs::msg::Pose::SharedPtr next_waypoint)
    {
        float tx = next_waypoint->position.x;
        float ty = next_waypoint->position.y;
        float desired_yaw = -1 * std::atan2(ty, tx);

        float steering_error = desired_yaw; // after mathmatical deduction, steering error = desired yaw
        return steering_error;
    }

    double PIDControlActionServer::yawFromOdom(nav_msgs::msg::Odometry::SharedPtr odom)
    {
        tf2::Quaternion quat(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
                             odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
        tf2::Matrix3x3 mat(quat);
        double yaw = atan2(mat[1][0], mat[0][0]);
        return yaw;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pid_controller::PIDControlActionServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
