#include "roar_septentrio_interface/roar_septentrio_interface_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "nav_msgs/msg/odometry.hpp"

namespace ROAR
{
  namespace SeptentrioInterface
  {
    SeptentrioInterfaceNode::SeptentrioInterfaceNode() : rclcpp::Node("septentrio_interface_node")
    {
      this->declare_parameter("debug", false);
      if (this->get_parameter("debug").as_bool())
      {
        auto ret = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
      }
      this->declare_parameter("map_frame", "map");
      this->declare_parameter("base_link_frame", "base_link");
      this->declare_parameter("datum", "0.0,0.0,0.0");
      this->declare_parameter("heading_offset_deg", 90.0);

      this->declare_parameter("should_publish_transform", false);

      // create a subscription with best effort QoS
      rclcpp::QoS qos(rclcpp::KeepLast(10));
      qos.best_effort();
      this->gps_subscriber_ = this->create_subscription<gps_msgs::msg::GPSFix>(
          "/gps/fix",
          qos,
          std::bind(&SeptentrioInterfaceNode::gps_callback, this, std::placeholders::_1));
      this->odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/roar/odometry", 10);
      this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
      this->parse_datum();
      RCLCPP_INFO_STREAM(get_logger(),
                         "\nInitialized with config\n\t Debug: " << this->get_parameter("debug").as_bool()
                                                                 << "\n\t Datum: "
                                                                 << this->get_parameter("datum").as_string()
                                                                 << "\n\t Map Frame: " << this->get_parameter("map_frame").as_string()
                                                                 << "\n\t Base Link Frame: " << this->get_parameter("base_link_frame").as_string());
    }

    SeptentrioInterfaceNode::~SeptentrioInterfaceNode()
    {
      // Destructor logic, if any
      RCLCPP_DEBUG(get_logger(), "SeptentrioInterfaceNode is destroyed.");
    }

    void SeptentrioInterfaceNode::gps_callback(const gps_msgs::msg::GPSFix::SharedPtr msg)
    {
      nav_msgs::msg::Odometry odom;
      odom.header.frame_id = this->get_parameter("map_frame").as_string();
      odom.header.stamp = this->now();
      odom.child_frame_id = this->get_parameter("base_link_frame").as_string();

      /**
       * Estimate Pose
       */
      GeodeticPosition inputGeoPosition;
      inputGeoPosition.latitude = msg->latitude;
      inputGeoPosition.longitude = msg->longitude;
      inputGeoPosition.altitude = msg->altitude;
      CartesianPosition outputCartesianPosition;
      convert_gnss_to_local_cartesian(inputGeoPosition, outputCartesianPosition);

      odom.pose.pose.position.x = float(outputCartesianPosition.x);
      odom.pose.pose.position.y = float(outputCartesianPosition.y);
      odom.pose.pose.position.z = float(outputCartesianPosition.z);
      if (!std::isnan(msg->dip))
      {
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), msg->dip));
      }
      else if (!(std::isnan(msg->track)))
      {
        // convert deg to rad
        double rad = (msg->track + this->get_parameter("heading_offset_deg").as_double()) * M_PI / 180.0;
        odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), rad));
      }

      /**
       * Estimate twist
       */
      odom.twist.twist.linear.x = msg->speed * cos(msg->track);
      odom.twist.twist.linear.y = msg->speed * sin(msg->track);
      odom.twist.twist.linear.z = 0;

      odom.twist.twist.angular.x = 0;
      odom.twist.twist.angular.y = 0;
      odom.twist.twist.angular.z = msg->climb;

      RCLCPP_DEBUG_STREAM(get_logger(), "\n-------------"
                                            << "\nGPS: " << msg->latitude << ", " << msg->longitude << ", " << msg->altitude
                                            << "\nCartesian: " << outputCartesianPosition.x << ", " << outputCartesianPosition.y << ", " << outputCartesianPosition.z);
      // << "\nPublishing Odometry message: "
      // << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y << ", " << odom.pose.pose.position.z);

      this->odom_publisher_->publish(odom);
      this->publishTransformFromOdom(std::make_shared<nav_msgs::msg::Odometry>(odom));
    }

    void SeptentrioInterfaceNode::publishTransformFromOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = this->get_parameter("map_frame").as_string();
      transformStamped.child_frame_id = this->get_parameter("base_link_frame").as_string();

      transformStamped.transform.translation.x = odom->pose.pose.position.x;
      transformStamped.transform.translation.y = odom->pose.pose.position.y;
      transformStamped.transform.translation.z = odom->pose.pose.position.z;

      transformStamped.transform.rotation = odom->pose.pose.orientation;

      // publish tf
      tf_broadcaster_->sendTransform(transformStamped);
    }

    void SeptentrioInterfaceNode::convert_gnss_to_local_cartesian(GeodeticPosition inputGeoPosition, CartesianPosition &outputCartesianPosition)
    {

      proj.Forward(inputGeoPosition.latitude,
                   inputGeoPosition.longitude,
                   inputGeoPosition.altitude,
                   outputCartesianPosition.x,
                   outputCartesianPosition.y,
                   outputCartesianPosition.z);
    }

    void
    SeptentrioInterfaceNode::parse_datum()
    {
      std::string datum_str = this->get_parameter("datum").as_string();

      // Split the string using ',' as the delimiter
      std::stringstream ss(datum_str);
      std::string token;
      std::vector<float> values;
      while (std::getline(ss, token, ','))
      {
        try
        {
          float value = std::stof(token);
          values.push_back(value);
        }
        catch (const std::exception &ex)
        {
          RCLCPP_ERROR(this->get_logger(), "Error parsing parameter: %s", ex.what());
        }
      }

      // Ensure we have exactly three values
      if (values.size() != 3)
      {
        RCLCPP_WARN(this->get_logger(), "Invalid parameter format for 'datum'. Expected 3 values separated by commas.");
      }
      else
      {
        map_origin_.latitude = values[0];
        map_origin_.longitude = values[1];
        map_origin_.altitude = values[2];
        const GeographicLib::Geocentric &earth = GeographicLib::Geocentric::WGS84();
        proj = GeographicLib::LocalCartesian(map_origin_.latitude, map_origin_.longitude, map_origin_.altitude, earth);
        RCLCPP_INFO(get_logger(), "Datum - Latitude: %.6f, Longitude: %.6f, Altitude: %.6f",
                    map_origin_.latitude, map_origin_.longitude, map_origin_.altitude);
      }
    }
  } // namespace SeptentrioInterface
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROAR::SeptentrioInterface::SeptentrioInterfaceNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
