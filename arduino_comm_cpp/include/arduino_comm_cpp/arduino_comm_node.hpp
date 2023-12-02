#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "roar_gokart_msgs/msg/actuation.hpp"
#include "roar_gokart_msgs/msg/vehicle_status.hpp"
#include "roar_gokart_msgs/msg/ego_vehicle_control.hpp"

#include "rapidjson/document.h"
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>



namespace roar
{
    namespace arduino
    {
        class ArduinoCommunicatorNode:  public rclcpp::Node
        {
            public:
                ArduinoCommunicatorNode();
                ~ArduinoCommunicatorNode();

            protected:
                // define read and write timer
                void on_read_timer();
                rclcpp::TimerBase::SharedPtr read_timer_;
                void on_write_timer();
                rclcpp::TimerBase::SharedPtr write_timer_;

                // define buffer to store received data
                std::array<char, 1024> buffer;
                const std::string message = "s";

                // Timer
                std::chrono::high_resolution_clock::time_point start_time;
                std::chrono::high_resolution_clock::time_point end_time;

                // define command subscriber 
                void on_command(const roar_gokart_msgs::msg::EgoVehicleControl::SharedPtr msg);
                rclcpp::Subscription<roar_gokart_msgs::msg::EgoVehicleControl>::SharedPtr command_subscriber_;

                // define state publisher
                void on_publish_state();
                rclcpp::TimerBase::SharedPtr publish_state_timer_;
                rclcpp::Publisher<roar_gokart_msgs::msg::VehicleStatus>::SharedPtr state_publisher_;

                void p_publish_state(const roar_gokart_msgs::msg::VehicleStatus::SharedPtr latest_state );

                // define memory pointer to the latest state and command 
                std::shared_ptr<roar_gokart_msgs::msg::VehicleStatus> latest_state_;
                std::shared_ptr<roar_gokart_msgs::msg::EgoVehicleControl> latest_command_;

                // define socket
                int sock;
                struct sockaddr_in server_address;
                struct in_addr arduino_ip;
                unsigned int arduino_port = 1883;

                // define function to get IP address and port
                std::pair<std::string, int> p_getIPAndPort();

                // define logger
                std::shared_ptr<rclcpp::Logger> _logger;

                // define control msg to arduino command function
                roar_gokart_msgs::msg::EgoVehicleControl p_egoVehicleControlMsgToArduinoCmdActionModel(const roar_gokart_msgs::msg::EgoVehicleControl::SharedPtr msg); 

                // define function to parse JSON data to vehicle state model
                roar_gokart_msgs::msg::VehicleStatus p_dataToVehicleState(const std::string& jsonData);  // const nlohmann::json& data as input

        };
    }
}