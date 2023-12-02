#ifndef ROAR__BEHAVIOR_PLANNING__ACTION_NODES__LAT_PID_TUNER_HPP_
#define ROAR__BEHAVIOR_PLANNING__ACTION_NODES__LAT_PID_TUNER_HPP_

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include <map>

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            namespace action
            {
                struct PidCoefficients
                {
                    double k_p = 1.0;
                    double k_i = 0.0;
                    double k_d = 0.0;
                };
                class LatPIDtuner : public BT::SyncActionNode
                {
                public:
                    LatPIDtuner(
                        const std::string &action_name,
                        const BT::NodeConfiguration &conf,
                        const rclcpp::Logger &logger,
                        rclcpp::Clock &clock,
                        rclcpp_lifecycle::LifecycleNode::SharedPtr node);
                    BT::NodeStatus tick() override;
                    static BT::PortsList providedPorts();
                    void p_readConfig();
                    void p_setParameter(float kp, float ki, float kd);

                private:
                    rclcpp::Logger logger_;
                    rclcpp::Clock &clock_;
                    std::map<int, PidCoefficients> pid_coefficients_;
                    rclcpp::Client<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_pid_client_;

                    rclcpp_lifecycle::LifecycleNode::SharedPtr node;
                };
            } // namespace action
        }     // namespace behavior
    }         // namespace planning
} // namespace roar

#endif // ROAR__BEHAVIOR_PLANNING__ACTION_NODES__LAT_PID_TUNER_HPP_