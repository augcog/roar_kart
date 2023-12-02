#ifndef PID_CONTROL_NODE_HPP_
#define PID_CONTROL_NODE_HPP_
#include <mutex>
#include <memory>
#include <thread>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <control_toolbox/pid_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_util/simple_action_server.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include <tf2_ros/transform_listener.h>


#include "control_interfaces/action/control.hpp"
using namespace std::placeholders;

namespace pid_controller
{
    class PIDControlActionServer : public nav2_util::LifecycleNode
    {
        public:
            static constexpr float DEFAULT_K_P = 0.05f;
            static constexpr float DEFAULT_K_D = 0.0f;
            static constexpr float DEFAULT_K_I = 0.0f;

            using ControlAction = control_interfaces::action::Control;
            using ActionServer = nav2_util::SimpleActionServer<ControlAction, rclcpp_lifecycle::LifecycleNode>;

            using GoalHandleControlAction = rclcpp_action::ServerGoalHandle<ControlAction>;

            explicit PIDControlActionServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
            ~PIDControlActionServer();

        private:
            nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
            nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
            nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
            nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                    std::shared_ptr<const ControlAction::Goal> goal);

            rclcpp_action::CancelResponse handle_cancel(
                const std::shared_ptr<GoalHandleControlAction> goal_handle);

            void handle_accepted(const std::shared_ptr<GoalHandleControlAction> goal_handle);

            void execute(const std::shared_ptr<GoalHandleControlAction> goal_handle);
            
            void onLatestOdomReceived(nav_msgs::msg::Odometry::SharedPtr msg);
            float calculateThrottleError(nav_msgs::msg::Odometry::SharedPtr odom, float target_spd);
            float calculateSteeringError(nav_msgs::msg::Odometry::SharedPtr odom, geometry_msgs::msg::Pose::SharedPtr next_waypoint);
            double yawFromOdom(nav_msgs::msg::Odometry::SharedPtr odom);
            float pSpeedFromOdom(nav_msgs::msg::Odometry::SharedPtr odom);
            int findNextWaypoint(const nav_msgs::msg::Path path, const std::shared_ptr<nav_msgs::msg::Odometry> odom);

            // geometry_msgs::msg::Pose findNextWaypoint(const nav_msgs::msg::Path path, const std::shared_ptr<nav_msgs::msg::Odometry> odom);
            void pPublishAckermannMsg(float steering_error, float steering_cmd, float throttle_cmd, std::string frame_id);
            void pPublishVisualizationFromPose(geometry_msgs::msg::Pose::SharedPtr pose);
            bool pReadPidFromFile();
            bool pLoadConfigs(const rapidjson::Value &config, std::map<float, control_toolbox::Pid::Gains> *configs);
            void pFollowWaypoint(rclcpp::Duration dt, const std::shared_ptr<GoalHandleControlAction> goal_handle);
            control_toolbox::Pid::Gains pConvertJsonToPid(const rapidjson::Value &config);
            void pPrettyPrintMap(const std::map<float, control_toolbox::Pid::Gains> *configs);

            control_toolbox::Pid::Gains pGainsFromSpeed(float spd, std::map<float, control_toolbox::Pid::Gains> *configs);
            void pUpdateLongAndLatPID(float spd);
            bool hasReachedGoalThreshold(const std::shared_ptr<GoalHandleControlAction> goal_handle);
            geometry_msgs::msg::Pose::SharedPtr pConvertToEgoFov(geometry_msgs::msg::Pose original);

            void
            printGains(const control_toolbox::Pid::Gains &gains)
            {
                RCLCPP_INFO(rclcpp::get_logger("pid_control_node"), "Gains: P=%.2f, I=%.2f, D=%.2f, i_max=%.2f, i_min=%.2f, antiwindup=%d",
                            gains.p_gain_, gains.i_gain_, gains.d_gain_, gains.i_max_, gains.i_min_, gains.antiwindup_);
            }

            /*********************** start of variable section ***********************/
            bool is_server_running = false;

            rclcpp_action::Server<ControlAction>::SharedPtr action_server_;
            std::shared_ptr<nav_msgs::msg::Odometry> latest_odom;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            std::mutex odom_mutex_;

            std::shared_ptr<control_toolbox::PidROS> longitudinal_pid;
            std::shared_ptr<rclcpp::Node> longitudinal_pid_node_;

            std::shared_ptr<control_toolbox::PidROS> lateral_pid;
            std::shared_ptr<rclcpp::Node> lateral_pid_node_;

            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>> ackermann_publisher_;
            std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>> next_waypoint_pub_;
            
            std::thread execution_thread_;
            std::atomic<bool> thread_mutex;

            std::map<float, control_toolbox::Pid::Gains> *longitudinal_pid_configs;
            std::map<float, control_toolbox::Pid::Gains> *lateral_pid_configs;

            std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
            std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

            float MAX_RIGHT_STEERING_ANGLE = 1.0;
            float MAX_LEFT_STEERING_ANGLE = -1.0;
            rclcpp::Time pid_last_calc_timestamp;
    };
}
#endif // PID_CONTROL_NODE_HPP_
