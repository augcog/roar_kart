#include <controller_manager/controller_manager_ros.hpp>
#include <controller_manager/pid_controller.hpp>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace controller
{
    ControllerManagerNode::ControllerManagerNode()
        : LifecycleNode("manager", "controller", true), m_plugin_loader_("controller_manager", "roar::control::ControllerPlugin")

    {
        m_config_ = std::make_shared<ControllerManagerConfig>(
            ControllerManagerConfig{
                declare_parameter<bool>("manager.debug", false),
                declare_parameter<double>("manager.loop_rate", 10.0),
                declare_parameter<double>("planner.target_speed", 10.0),
                declare_parameter<double>("planner.max_speed", 10.0),
                declare_parameter<std::string>("base_link_frame", "base_link"),
                declare_parameter<std::string>("map_frame", "map"),
                declare_parameter<double>("min_distance", 5.0)});

        if (m_config_->debug)
        {
            bool _ = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                      RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
            _ = rcutils_logging_set_logger_level("controller.manager.rclcpp_action",
                                                 RCUTILS_LOG_SEVERITY_FATAL); // enable or disable debug
        }
        // initialize plugins
        const auto plugin_names = declare_parameter("plugins", std::vector<std::string>{});
        RCLCPP_INFO_STREAM(this->get_logger(), "plugin_names: " << plugin_names.size() << " plugins");
        for (const auto &plugin_name : plugin_names)
        {
            roar::control::ControllerPlugin::SharedPtr new_plugin = m_plugin_loader_.createSharedInstance(plugin_name);
            m_plugins_.push_back(new_plugin);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "plugin: " << plugin_name << " loaded");
        }
        std::for_each(
            m_plugins_.begin(), m_plugins_.end(), [this](roar::control::ControllerPlugin::SharedPtr &p)
            { 
                p->initialize(this);
                RCLCPP_DEBUG_STREAM(this->get_logger(), "plugin: " << p->get_plugin_name() << " initialized"); });
        std::for_each(
            m_plugins_.begin(), m_plugins_.end(), [this](roar::control::ControllerPlugin::SharedPtr &p)
            { 
                p->configure(m_config_);
                RCLCPP_DEBUG_STREAM(this->get_logger(), "plugin: " << p->get_plugin_name() << " configured"); });

        // initialize state
        m_controller_state_ = std::make_shared<ControllerManagerState>(ControllerManagerState());

        RCLCPP_INFO(this->get_logger(),
                    "ControllerManagerNode initialized with Debug Mode = [%s]",
                    m_config_->debug ? "YES" : "NO");
    }

    ControllerManagerNode ::~ControllerManagerNode()
    {
        if (this->execution_timer)
        {
            this->execution_timer->cancel();
        }
    }

    /**
     * Lifecycles
     */
    nav2_util::CallbackReturn
    ControllerManagerNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(get_logger(), "on_configure");
        // behavior
        this->behavior_status_sub_ = this->create_subscription<roar_msgs::msg::BehaviorStatus>(
            "behavior_status", 10,
            std::bind(&ControllerManagerNode::behavior_status_callback, this,
                      std::placeholders::_1));

        // action server
        this->action_server_ = rclcpp_action::create_server<ControlAction>(
            this,
            std::string(this->get_namespace()) + "/" +
                std::string(this->get_name()),
            std::bind(&ControllerManagerNode::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&ControllerManagerNode::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&ControllerManagerNode::handle_accepted, this,
                      std::placeholders::_1));
        this->execution_timer = this->create_wall_timer(
            std::chrono::milliseconds(int(this->m_config_->loop_rate_millis)),
            std::bind(&ControllerManagerNode::execution_callback, this));

        // safety switch
        this->control_safety_switch_ = this->create_service<roar_msgs::srv::ToggleControlSafetySwitch>(std::string(this->get_namespace()) + "/" +
                                                                                                           std::string(this->get_name()) + "/safety_toggle",
                                                                                                       std::bind(&ControllerManagerNode::toggle_safety_switch, this, std::placeholders::_1, std::placeholders::_2));
        // diagnostic
        diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
            "/diagnostics", 10);
        // tf buffer
        tf_buffer_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // output publisher
        this->vehicle_control_publisher_ = this->create_publisher<roar_msgs::msg::VehicleControl>("vehicle_control", 10);

        // vehicle_state listener
        this->vehicle_state_sub_ = this->create_subscription<roar_msgs::msg::VehicleState>(
            "vehicle_state", 10,
            std::bind(&ControllerManagerNode::vehicle_state_callback, this,
                      std::placeholders::_1));
        RCLCPP_DEBUG(get_logger(), "configured");

        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn
    ControllerManagerNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(get_logger(), "on_activate");
        this->vehicle_control_publisher_->on_activate();
        this->diagnostic_pub_->on_activate();
        RCLCPP_DEBUG(get_logger(), "activated");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn
    ControllerManagerNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(get_logger(), "on_deactivate");
        this->vehicle_control_publisher_->on_deactivate();
        this->diagnostic_pub_->on_deactivate();
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn
    ControllerManagerNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(get_logger(), "on_cleanup");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn
    ControllerManagerNode::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_DEBUG(get_logger(), "on_shutdown");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    /**
     * Action server
     */
    rclcpp_action::GoalResponse ControllerManagerNode::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ControlAction::Goal> goal)
    {
        auto diag_array_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diag_array_msg->header.stamp = this->now();
        diagnostic_msgs::msg::DiagnosticStatus::SharedPtr diag_status_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
        diag_status_msg->name = std::string(this->get_namespace()) + "/" +
                                std::string(this->get_name());

        if (this->is_auto_control == false)
        {
            diag_status_msg->level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag_status_msg->message = "rejecting goal - Auto is off";
            diag_array_msg->status.push_back(*diag_status_msg);
            diagnostic_pub_->publish(std::move(diag_array_msg));

            return rclcpp_action::GoalResponse::REJECT;
        }
        if (this->active_goal_ != nullptr)
        {
            diag_status_msg->level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag_status_msg->message = "rejecting goal - goal already in progress";
            diag_array_msg->status.push_back(*diag_status_msg);
            diagnostic_pub_->publish(std::move(diag_array_msg));
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->path.poses.size() == 0)
        {
            diag_status_msg->level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag_status_msg->message = "rejecting goal - path is empty";
            diag_array_msg->status.push_back(*diag_status_msg);
            diagnostic_pub_->publish(std::move(diag_array_msg));
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ControllerManagerNode::handle_cancel(
        const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ControllerManagerNode::handle_accepted(
        const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        active_goal_ = goal_handle;
    }

    /**
     * main execution loops
     */
    void ControllerManagerNode::execution_callback()
    {
        if (this->is_auto_control == false)
        {
            auto diag_array_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
            diag_array_msg->header.stamp = this->now();
            diagnostic_msgs::msg::DiagnosticStatus::SharedPtr diag_status_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
            diag_status_msg->name = "ControllerManager Execution";
            diag_array_msg->status.push_back(*diag_status_msg);
            diag_status_msg->level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            diag_status_msg->message = "Auto is off";
            diagnostic_pub_->publish(std::move(diag_array_msg));

            roar_msgs::msg::VehicleControl neutralControlMsg;
            neutralControlMsg.header.stamp = this->now();
            neutralControlMsg.header.frame_id = m_config_->base_link_frame;
            neutralControlMsg.target_speed = 0.0;
            neutralControlMsg.steering_angle = 0.0;
            neutralControlMsg.is_auto = false;

            this->vehicle_control_publisher_->publish(neutralControlMsg);
            return;
        }

        if (this->active_goal_ != nullptr)
        {
            this->p_execute(active_goal_);
        }
    }

    void ControllerManagerNode::behavior_status_callback(const roar_msgs::msg::BehaviorStatus::SharedPtr msg)
    {
        m_controller_state_->behavior_status = msg;
        this->on_update();
    }

    void ControllerManagerNode::on_update()
    {
        std::for_each(
            m_plugins_.begin(), m_plugins_.end(), [this](roar::control::ControllerPlugin::SharedPtr &p)
            { p->update(m_controller_state_); });
    }

    void ControllerManagerNode::p_execute(
        const std::shared_ptr<GoalHandleControlAction> goal_handle)
    {
        RCLCPP_DEBUG(get_logger(), "------ controller manager ------");
        auto diag_array_msg = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diag_array_msg->header.stamp = this->now();
        diagnostic_msgs::msg::DiagnosticStatus::SharedPtr diag_status_msg = std::make_shared<diagnostic_msgs::msg::DiagnosticStatus>();
        diag_status_msg->name = "ControllerManager Execution";
        diag_array_msg->status.push_back(*diag_status_msg);

        std::lock_guard<std::mutex> lock(active_goal_mutex_);
        // get path from goal
        nav_msgs::msg::Path path = goal_handle->get_goal()->path;

        // transform path to ego centric frame
        RCLCPP_DEBUG_STREAM(get_logger(), "global_path[0]: " << path.poses[0].pose.position.x << " "
                                                              << path.poses[0].pose.position.y << " ");
        nav_msgs::msg::Path egoCentricPath = this->p_transformToEgoCentric(path);
        RCLCPP_DEBUG_STREAM(get_logger(), "egoCentricPath: " << egoCentricPath.poses.size() << " poses");
        if (egoCentricPath.poses.size() == 0)
        {
            diag_status_msg->message = "rejecting goal - path is empty or failed to transform to target frame";
            diag_status_msg->level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diagnostic_pub_->publish(std::move(diag_array_msg));
            RCLCPP_ERROR(get_logger(), "rejecting goal - path is empty or failed to transform to target frame");
            auto result = std::make_shared<ControlAction::Result>();
            result->status = control_interfaces::action::Control::Result::FAILED;
            goal_handle->succeed(result);
            active_goal_ = nullptr; // release lock
            return;
        }

        m_controller_state_->path_ego_centric = egoCentricPath;

        /**
         * All updates are parsed, now update controllers
         */
        bool all_update_good = std::all_of(
            m_plugins_.begin(), m_plugins_.end(), [this](roar::control::ControllerPlugin::SharedPtr &p)
            {
                try {
                    bool status = p->update(m_controller_state_);
                    if (status == false)
                    {
                        RCLCPP_ERROR(get_logger(), "plugin %s failed to update", p->get_plugin_name());
                    }
                    
                    return status;
                }
                catch (const std::exception &e) {
                    RCLCPP_ERROR(get_logger(), "plugin %s failed to update: %s", p->get_plugin_name(), e.what());
                    return false;
                } });

        if (!all_update_good)
        {
            RCLCPP_ERROR(get_logger(), "rejecting goal - plugin failed to update");
            auto result = std::make_shared<ControlAction::Result>();
            result->status = control_interfaces::action::Control::Result::FAILED;
            goal_handle->succeed(result);
            active_goal_ = nullptr; // release lock
            return;
        }

        // all controllers are updated, now compute
        // construct control msg
        roar_msgs::msg::VehicleControl::SharedPtr controlMsg = std::make_shared<roar_msgs::msg::VehicleControl>();
        controlMsg->header.stamp = this->now();
        controlMsg->header.frame_id = m_config_->base_link_frame;

        // find all controls by running through a list of plugins
        bool output_good = std::all_of(
            m_plugins_.begin(), m_plugins_.end(), [controlMsg, this](roar::control::ControllerPlugin::SharedPtr &p)
            {
                try {
                    bool status = p->compute(controlMsg);
                    if (status == false)
                    {
                        RCLCPP_ERROR(get_logger(), "plugin %s failed to compute", p->get_plugin_name());
                    }

                    return status;
                    }
                catch (const std::exception &e) {
                    RCLCPP_ERROR(get_logger(), "plugin %s failed to compute: %s", p->get_plugin_name(), e.what());
                    return false;
                } });

        // if any output is no good
        if (!output_good)
        {
            RCLCPP_ERROR(get_logger(), "rejecting goal - plugin failed to step");
            auto result = std::make_shared<ControlAction::Result>();
            result->status = control_interfaces::action::Control::Result::FAILED;
            goal_handle->succeed(result);
            active_goal_ = nullptr; // release lock
            return;
        }
        controlMsg->is_auto = true;

        RCLCPP_DEBUG_STREAM(get_logger(), "TargetSpeed: " << controlMsg->target_speed << " SteeringAngle: "
                                                          << controlMsg->steering_angle << " Brake: " << controlMsg->brake);
        // publish control
        this->vehicle_control_publisher_->publish(*controlMsg);

        auto result = std::make_shared<ControlAction::Result>();
        result->status = control_interfaces::action::Control::Result::NORMAL;
        goal_handle->succeed(result);
        active_goal_ = nullptr; // release lock
        return;
    }

    nav_msgs::msg::Path ControllerManagerNode::p_transformToEgoCentric(nav_msgs::msg::Path path)
    {
        std::string target_frame = this->get_parameter("base_link_frame").as_string();
        std::string fromFrameRel = path.header.frame_id == "" ? this->get_parameter("map_frame").as_string() : path.header.frame_id;
        std::string toFrameRel = target_frame;

        // Get the current pose of the robot (ego vehicle) in the target frame
        geometry_msgs::msg::PoseStamped ego_pose;
        ego_pose.header.frame_id = target_frame;
        ego_pose.pose.orientation.w = 1.0; // Assuming the orientation is identity

        nav_msgs::msg::Path transformed_path;

        // Transform each pose in the path to the ego-centric frame
        for (auto &pose : path.poses)
        {
            try
            {
                geometry_msgs::msg::TransformStamped t;
                t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel, tf2::TimePointZero);
                geometry_msgs::msg::PoseStamped transformOut;

                tf2::doTransform(pose, transformOut, t);
                transformed_path.poses.push_back(transformOut); // Add the transformed pose to the new path
            }
            catch (tf2::TransformException &ex)
            {
                // Handle the exception if the transform is not available
                // (e.g., if the required transformation is not in the tf tree)
                // You can choose to skip or abort the transformation for this pose.
                RCLCPP_WARN(this->get_logger(), "Failed to transform pose: %s", ex.what());
            }
        }
        return transformed_path;
    }


    void ControllerManagerNode::toggle_safety_switch(const std::shared_ptr<roar_msgs::srv::ToggleControlSafetySwitch::Request> request,
                                                     std::shared_ptr<roar_msgs::srv::ToggleControlSafetySwitch::Response> response)
    {
        this->is_auto_control = request->is_safety_on;
        response->status = this->is_auto_control;

        RCLCPP_INFO(this->get_logger(), "Auto switch is %s", this->is_auto_control ? "ON" : "OFF");
    }
} // namespace controller
