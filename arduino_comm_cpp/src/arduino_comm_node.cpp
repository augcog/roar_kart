#include "arduino_comm_cpp/arduino_comm_node.hpp"

namespace roar {
    namespace arduino {
        
        // Constructor
        ArduinoCommunicatorNode::ArduinoCommunicatorNode() : Node("arduino_comm_node"){

            // Declare ROS parameters for IP address, port, and timings
            this->declare_parameter("ip_address", "10.0.0.9");
            this->declare_parameter("port", 1883);
            this->declare_parameter("get_state_period", 0.001);
            this->declare_parameter("write_action_period", 0.001);
            this->declare_parameter("write_timeout", 0.1);
            this->declare_parameter("vehicle_status_header", "base_link");

            // Retrieve parameters and create timers based on them
            double get_state_period = this->get_parameter("get_state_period").as_double();
            this->read_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(get_state_period),
                std::bind(&ArduinoCommunicatorNode::on_read_timer, this)
            );

            double write_action_period = this->get_parameter("write_action_period").as_double();
            this->write_timer_ = this->create_wall_timer(
                std::chrono::duration<double>(write_action_period),
                std::bind(&ArduinoCommunicatorNode::on_write_timer, this)
            );

            // Create subscription and publisher
            command_subscriber_ = this->create_subscription<roar_gokart_msgs::msg::EgoVehicleControl>(
                "ego_vehicle_control",
                10,
                std::bind(&ArduinoCommunicatorNode::on_command, this, std::placeholders::_1)
            );
            state_publisher_ = this->create_publisher<roar_gokart_msgs::msg::VehicleStatus>(
                "vehicle_status",
                10
            );

            // initialize latest state and latest command
            latest_state_ = std::make_shared<roar_gokart_msgs::msg::VehicleStatus>();
            latest_command_ = std::make_shared<roar_gokart_msgs::msg::EgoVehicleControl>();

            // Set up the UDP socket
            sock = socket(AF_INET, SOCK_DGRAM, 0);

            arduino_ip.s_addr = inet_addr("10.0.0.9");
            server_address.sin_family = AF_INET;
            server_address.sin_addr = arduino_ip;
            server_address.sin_port = htons(arduino_port);

            // Log message
            RCLCPP_INFO(get_logger(), "ArduinoCommunicatorNode has been initialized");
        }
        ArduinoCommunicatorNode::~ArduinoCommunicatorNode()
        {
            // Close socket
            close(sock);
        }


        // Function to get state data from Arduino
        void ArduinoCommunicatorNode::on_read_timer() {

            // Send message via UDP (message = 's')
            sendto(sock, message.c_str(), message.size(), 0, (struct sockaddr*)&server_address, sizeof(server_address));            


            // Receive data from socket with a buffer size of 1024 bytes
            struct sockaddr_in client_address;
            socklen_t client_address_len = sizeof(client_address);
            ssize_t num_bytes_received = recvfrom(ArduinoCommunicatorNode::sock, buffer.data(), buffer.size(), 0, (struct sockaddr*)&client_address, &client_address_len);



            // Add null terminator to received message
            buffer[num_bytes_received] = '\0';
            try {
                auto model = p_dataToVehicleState(buffer.data());
                *latest_state_ = model;

            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "Failed to parse received data: %s", e.what());
            }


        }

        // Callback function for sending control commands to Arduino
        void ArduinoCommunicatorNode::on_write_timer() {
            std::string message;

            // Create a RapidJSON document
            rapidjson::Document doc;
            doc.SetObject();

            // Create a RapidJSON array for the current actuation
            doc.AddMember("target_speed", latest_command_->target_speed, doc.GetAllocator());
            doc.AddMember("steering_angle", latest_command_->steering_angle, doc.GetAllocator());
            doc.AddMember("brake", latest_command_->brake, doc.GetAllocator());
            doc.AddMember("reverse", latest_command_->reverse, doc.GetAllocator());
            
            // Serialize the document to a JSON string
            rapidjson::StringBuffer buffer;
            rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
            doc.Accept(writer);
            std::string json_string_data = buffer.GetString();

            // send message via UDP
            sendto(sock, json_string_data.c_str(), json_string_data.size(), 0, (struct sockaddr*)&server_address, sizeof(server_address));


        }

        // Callback function for receiving EgoVehicleControl commands from ROS
        void ArduinoCommunicatorNode::on_command(const roar_gokart_msgs::msg::EgoVehicleControl::SharedPtr msg) {
            auto command = roar_gokart_msgs::msg::EgoVehicleControl();
            command = p_egoVehicleControlMsgToArduinoCmdActionModel(msg);

            // update latest command
            *latest_command_ = command;
            
        }


        // Callback function for publishing the current state
        void ArduinoCommunicatorNode::on_publish_state() {
            // Create header for the message
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = this->get_parameter("vehicle_status_header").as_string();

            // Create actuation for the message
            roar_gokart_msgs::msg::Actuation act;
            act.brake = latest_command_->brake;
            act.reverse = latest_command_->reverse;
            act.steering = latest_command_->steering_angle;
            act.throttle = latest_command_->target_speed;

            // Create vehicle status message
            roar_gokart_msgs::msg::VehicleStatus msg;
            msg.header = header;
            msg.actuation = act;
            msg.angle = latest_state_->angle;
            msg.speed = latest_state_->speed;
            msg.is_left_limiter_on = latest_state_->is_left_limiter_on;
            msg.is_right_limiter_on = latest_state_->is_right_limiter_on;
            msg.target_speed = latest_state_->target_speed; 
            msg.target_steering_angle = latest_state_->target_steering_angle; 

            // Publish the vehicle status message
            state_publisher_->publish(msg);
        }

        // Save the EgoVehicleControl message (pointer) in model
        roar_gokart_msgs::msg::EgoVehicleControl ArduinoCommunicatorNode::p_egoVehicleControlMsgToArduinoCmdActionModel(const roar_gokart_msgs::msg::EgoVehicleControl::SharedPtr msg) {
            roar_gokart_msgs::msg::EgoVehicleControl model;
            // Clip the brake value to ensure it's within the valid range [0, 1]
            model.brake = std::clamp(msg->brake, 0.0f, 1.0f);
            // Copy the reverse, steering angle, and target speed from the received message
            model.reverse = msg->reverse;
            model.steering_angle = msg->steering_angle;
            model.target_speed = msg->target_speed;

            return model;
        }


        // Parse JSON data to vehicle state model
        roar_gokart_msgs::msg::VehicleStatus ArduinoCommunicatorNode::p_dataToVehicleState(const std::string& jsonData) { 
            roar_gokart_msgs::msg::VehicleStatus model;
            rapidjson::Document document;
            document.Parse(jsonData.c_str());

            // Parse individual fields and populate the model
            model.is_auto = document["is_auto"].GetBool();
            model.is_left_limiter_on = document["is_left_limiter_ON"].GetBool();
            model.is_right_limiter_on = document["is_right_limiter_ON"].GetBool();
            model.angle = document["angle"].GetDouble();
            model.speed = document["speed"].GetDouble();
            model.target_speed = document["target_speed"].GetDouble();
            model.target_steering_angle = document["target_steering_angle"].GetDouble();

            const rapidjson::Value& currentActuation = document["current_actuation"];
            model.actuation.throttle = currentActuation["throttle"].GetDouble();
            model.actuation.steering = currentActuation["steering"].GetDouble();
            model.actuation.brake = currentActuation["brake"].GetDouble();
            model.actuation.reverse = currentActuation["reverse"].GetBool();

            // Publish the current state
            p_publish_state(std::make_shared<roar_gokart_msgs::msg::VehicleStatus>(model)); 

            return model;
        }

        // Callback function for publishing the current state
        void ArduinoCommunicatorNode::p_publish_state(const roar_gokart_msgs::msg::VehicleStatus::SharedPtr latest_state_ ) {
            // Create Header
            std_msgs::msg::Header header;
            header.stamp = rclcpp::Clock().now(); 
            header.frame_id = this->get_parameter("vehicle_status_header").as_string();

            // Create Actuation
            roar_gokart_msgs::msg::Actuation act;
            act.throttle = latest_state_->actuation.throttle;
            act.steering = latest_state_->actuation.steering;
            act.brake = latest_state_->actuation.brake;
            act.reverse = latest_state_->actuation.reverse;

            // Create VehicleStatus message
            roar_gokart_msgs::msg::VehicleStatus msg;
            msg.header = header;
            msg.angle = latest_state_->angle;
            msg.is_auto = latest_state_->is_auto;
            msg.is_left_limiter_on = latest_state_->is_left_limiter_on;
            msg.is_right_limiter_on = latest_state_->is_right_limiter_on;
            msg.speed = latest_state_->speed;
            msg.target_speed = latest_state_->target_speed;
            msg.target_steering_angle = latest_state_->target_steering_angle;
            msg.actuation = act;

            // Publish the message
            this->state_publisher_->publish(msg);

        }

        
        // Get IP address and port from parameters
        std::pair<std::string, int> ArduinoCommunicatorNode::p_getIPAndPort() {
            auto ip_address = this->get_parameter("ip_address").as_string();
            auto port = this->get_parameter("port").as_int();

            return std::make_pair(ip_address, port);
        }

    }  // namespace arduino
}  // namespace roar


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roar::arduino::ArduinoCommunicatorNode>());
  rclcpp::shutdown();
  return 0;
}