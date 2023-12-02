#include "behavior_planning/bt_nodes/action_nodes/lat_pid_tuner.hpp"
#include "rclcpp/logging.hpp"
#include "behavior_planning/common/utils.hpp"

#include "rapidjson/document.h"

#include <iostream>
#include <fstream>
#include "rapidjson/document.h"

using namespace rapidjson;

namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            namespace action
            {

                LatPIDtuner::LatPIDtuner(
                    const std::string &action_name,
                    const BT::NodeConfiguration &conf,
                    const rclcpp::Logger &logger,
                    rclcpp::Clock &clock,
                    rclcpp_lifecycle::LifecycleNode::SharedPtr node) : BT::SyncActionNode(action_name, conf), logger_(logger), clock_(clock), node(node)
                {
                    RCLCPP_DEBUG(logger_, "LatPIDtuner created");
                    set_pid_client_ = node->create_client<rcl_interfaces::srv::SetParametersAtomically>("controller_set_param_service"); // E.g.: serviceName = "/turtlesim/set_parameters_atomically"

                    p_readConfig();
                }

                void LatPIDtuner::p_readConfig()
                {
                    // get the file path from blackboard
                    BT::Optional<std::string> file_path = getInput<std::string>("pid_file_path");
                    if (!file_path)
                    {
                        RCLCPP_ERROR(logger_, "BehaviorPlannerBTLifeCycleNode: no file path");
                    }

                    std::ifstream file(file_path.value());
                    if (!file.is_open())
                    {
                        RCLCPP_ERROR_STREAM(logger_, "BehaviorPlannerBTLifeCycleNode: [" << file_path.value() << "] file not found");
                    }

                    std::string jsonData((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                    // Parse the JSON data
                    Document document;
                    if (document.Parse(jsonData.c_str()).HasParseError())
                    {
                        RCLCPP_ERROR(logger_, "BehaviorPlannerBTLifeCycleNode: parse error");
                    }

                    // Access the elements in the JSON data
                    const Value &latitudinalController = document["latitudinal_controller"];

                    for (Value::ConstMemberIterator it = latitudinalController.MemberBegin(); it != latitudinalController.MemberEnd(); ++it)
                    {
                        const int key = std::stoi(it->name.GetString());
                        const Value &controller = it->value;
                        const double k_p = controller["k_p"].GetDouble();
                        const double k_d = controller["k_d"].GetDouble();
                        const double k_i = controller["k_i"].GetDouble();

                        // RCLCPP_INFO_STREAM(logger_, "BehaviorPlannerBTLifeCycleNode: key: [" << key << "], k_p: [" << k_p << "], k_d: [" << k_d << "], k_i: [" << k_i << "]");
                        pid_coefficients_[key] = PidCoefficients{k_p, k_i, k_d};
                    }

                    // print all pid_coefficients
                    for (auto const &x : pid_coefficients_)
                    {
                        RCLCPP_INFO_STREAM(logger_, "BehaviorPlannerBTLifeCycleNode: key: [" << x.first << "], k_p: [" << x.second.k_p << "], k_d: [" << x.second.k_d << "], k_i: [" << x.second.k_i << "]");
                    }
                }

                BT::NodeStatus LatPIDtuner::tick()
                {
                    RCLCPP_DEBUG(logger_, "LatPIDtuner actuating...");
                    BT::Optional<roar::planning::behavior::BTOutputs::SharedPtr> outputs = config().blackboard->get<roar::planning::behavior::BTOutputs::SharedPtr>("outputs");
                    if (!outputs)
                    {
                        RCLCPP_ERROR(logger_, "BehaviorPlannerBTLifeCycleNode: no outputs");
                        return BT::NodeStatus::FAILURE;
                    }

                    // get vehicle_speed
                    auto inputs = config().blackboard->get<roar::planning::behavior::BTInputs::ConstSharedPtr>("inputs");
                    if (!inputs)
                    {
                        RCLCPP_ERROR(logger_, "LatPIDtuner: no inputs");
                        return BT::NodeStatus::FAILURE;
                    }

                    if (!inputs->vehicle_state)
                    {
                        RCLCPP_WARN(logger_, "LatPIDtuner: no vehicle state");
                        return BT::NodeStatus::FAILURE;
                    }

                    double speed = inputs->vehicle_state->vehicle_status.speed;

                    // loop through pid_coefficients_
                    // if vehicle_speed <= key, use the value corresponding to that key
                    // set param via c++
                    // return BT::NodeStatus::SUCCESS;
                    for (auto const &x : pid_coefficients_)
                    {
                        if (speed <= x.first)
                        {
                            p_setParameter(x.second.k_p, x.second.k_i, x.second.k_d);
                            RCLCPP_DEBUG(logger_, "LatPIDtuner: speed: [%f] -> k_p: [%f], k_d: [%f], k_i: [%f]", speed, x.second.k_p, x.second.k_d, x.second.k_i);
                            return BT::NodeStatus::SUCCESS;
                        }
                    }

                    return BT::NodeStatus::SUCCESS;
                }

                void LatPIDtuner::p_setParameter(float kp, float ki, float kd)
                {
                    auto request = std::make_shared<rcl_interfaces::srv::SetParametersAtomically::Request>();

                    auto kp_param = rcl_interfaces::msg::Parameter();
                    kp_param.name = "lat_control.pid.kp";
                    kp_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                    kp_param.value.double_value = kp;
                    request->parameters.push_back(kp_param);

                    auto kd_param = rcl_interfaces::msg::Parameter();
                    kd_param.name = "lat_control.pid.kd";
                    kd_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                    kd_param.value.double_value = kd;
                    request->parameters.push_back(kd_param);

                    auto ki_param = rcl_interfaces::msg::Parameter();
                    ki_param.name = "lat_control.pid.ki";
                    ki_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
                    ki_param.value.double_value = ki;
                    request->parameters.push_back(ki_param);

                    auto result = set_pid_client_->async_send_request(request);
                }

                BT::PortsList LatPIDtuner::providedPorts()
                {
                    return {
                        BT::InputPort<std::string>("pid_file_path"),
                        BT::InputPort<roar::planning::behavior::BTInputs::ConstSharedPtr>("inputs"),
                        BT::OutputPort<roar::planning::behavior::BTOutputs::SharedPtr>("outputs")};
                }
            }
        }
    }
} // namespace roar
