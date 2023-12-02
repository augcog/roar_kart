#include "roar_robot_localization/localization_hack.hpp"
#include <rmw/qos_profiles.h>
#include "rmw/types.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <sstream>

namespace roar
{

  LocalizationHack::LocalizationHack() : Node("localization_hack")
  {
    // variables
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("base_link_frame", "base_link");
    this->declare_parameter("rate_millis", 50);
    this->declare_parameter("datum", "0.0,0.0,0.0");
    this->declare_parameter("min_dist", 0.01);

    this->declare_parameter("debug", false);
    if (this->get_parameter("debug").as_bool())
    {
      auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                  RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }

    this->parse_datum();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();
    gps_sub_ = this->create_subscription<gps_msgs::msg::GPSFix>(
        "/gps/gps", qos, std::bind(&LocalizationHack::topic_callback, this, _1));

    odom_publisher_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/output/odom", 10);

    this->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

  LocalizationHack::~LocalizationHack()
  {
    // Perform any necessary cleanup here
  }

  void LocalizationHack::topic_callback(const gps_msgs::msg::GPSFix::SharedPtr gps_msg)
  {
    std::shared_ptr<nav_msgs::msg::Odometry> odom = this->computeOdomFromGps(gps_msg);

    // publish odom
    this->odom_publisher_->publish(*odom);

    // publish transform
    this->publishTransformFromOdom(odom);
    
    // // convert coordinate to cartesian space
    // CartesianPosition cartesian_position;
    // convert_gnss_to_local_cartesian(gps_msg, cartesian_position);

    // // check if this data can be used to compute steering angle
    // // if yes, memorize this
    // // if not, do nothing
    // if (this->latest_cartesian_used_for_steering_ == nullptr)
    // {
    //   this->latest_cartesian_used_for_steering_ = std::make_shared<CartesianPosition>(cartesian_position);
    //   return;
    // }

    // RCLCPP_DEBUG(get_logger(), "---------------------");

    // geometry_msgs::msg::TransformStamped transformStamped;
    // transformStamped.header.stamp = this->now();
    // transformStamped.header.frame_id = this->get_parameter("map_frame").as_string();
    // transformStamped.child_frame_id = this->get_parameter("base_link_frame").as_string();

    // // print latest_cartesian_used_for_steering
    // RCLCPP_DEBUG(this->get_logger(), "last: [%.6f, %.6f, %.6f]",
    //              this->latest_cartesian_used_for_steering_->x,
    //              this->latest_cartesian_used_for_steering_->y,
    //              this->latest_cartesian_used_for_steering_->z);
    // // print cartesian_pos
    // RCLCPP_DEBUG(this->get_logger(), "curr: [%.6f, %.6f, %.6f]",
    //              cartesian_position.x,
    //              cartesian_position.y,
    //              cartesian_position.z);

    // // compute position
    // transformStamped.transform.translation.x = cartesian_position.x;
    // transformStamped.transform.translation.y = cartesian_position.y;
    // transformStamped.transform.translation.z = cartesian_position.z;

    // if (this->is_steering_angle_computable(cartesian_position))
    // {
    //   // compute steering angle
    //   latest_steering_angle_ = std::atan2(cartesian_position.y - this->latest_cartesian_used_for_steering_->y,
    //                                       cartesian_position.x - this->latest_cartesian_used_for_steering_->x);
    //   RCLCPP_DEBUG(this->get_logger(), "angle: %.6f", latest_steering_angle_);
    // }
    // else
    // {
    //   RCLCPP_DEBUG(this->get_logger(), "Assuming prev steering angle [%f]", latest_steering_angle_);
    // }
    // geometry_msgs::msg::Quaternion orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), latest_steering_angle_));

    // transformStamped.transform.rotation = orientation;
    // // publish tf
    // tf_broadcaster_->sendTransform(transformStamped);

    // // publish odom
    // this->publishOdom(std::make_shared<geometry_msgs::msg::TransformStamped>(transformStamped));

    // RCLCPP_DEBUG(this->get_logger(), "GNSS: [%.6f, %.6f, %.6f]", gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    // RCLCPP_DEBUG(this->get_logger(), "local_coord: [%.6f, %.6f, %.6f]", cartesian_position.x, cartesian_position.y, cartesian_position.z);
    // this->latest_cartesian_used_for_steering_ = std::make_shared<CartesianPosition>(cartesian_position);
  }
  std::shared_ptr<nav_msgs::msg::Odometry> LocalizationHack::computeOdomFromGps(const gps_msgs::msg::GPSFix::SharedPtr gps_msg)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = this->get_parameter("map_frame").as_string();
    odom.child_frame_id = this->get_parameter("base_link_frame").as_string();

    // convert coordinate to cartesian space
    CartesianPosition cartesian_position;
    convert_gnss_to_local_cartesian(gps_msg, cartesian_position);

    // compute position
    odom.pose.pose.position.x = cartesian_position.x;
    odom.pose.pose.position.y = cartesian_position.y;
    odom.pose.pose.position.z = cartesian_position.z;

    if (last_odom_ == nullptr)
    {
      last_odom_ = std::make_shared<nav_msgs::msg::Odometry>(odom);
      return last_odom_;
    }

    // if the current odom is far enough away
    float dist = std::sqrt(std::pow(odom.pose.pose.position.x - last_odom_->pose.pose.position.x, 2) + 
                           std::pow(odom.pose.pose.position.y - last_odom_->pose.pose.position.y, 2));
    if (dist < this->get_parameter("min_dist").as_double()) {
      RCLCPP_WARN(this->get_logger(), "Too close to give an estimate");
      return last_odom_;
    } 

    double deltaTime = static_cast<double>(odom.header.stamp.sec - last_odom_->header.stamp.sec) +
                       static_cast<double>(odom.header.stamp.nanosec - last_odom_->header.stamp.nanosec) * 1e-9;

    // compute velocity
    odom.twist.twist.linear.x = (odom.pose.pose.position.x - last_odom_->pose.pose.position.x) / deltaTime;
    odom.twist.twist.linear.y = (odom.pose.pose.position.y - last_odom_->pose.pose.position.y) / deltaTime;
    odom.twist.twist.linear.z = (odom.pose.pose.position.z - last_odom_->pose.pose.position.z) / deltaTime;

    //compute angular velocity
    odom.twist.twist.angular.z = std::atan2(odom.pose.pose.position.y - last_odom_->pose.pose.position.y,
                                            odom.pose.pose.position.x - last_odom_->pose.pose.position.x) / deltaTime;

    // compute orientation
    float angle = std::atan2(odom.pose.pose.position.y - last_odom_->pose.pose.position.y,
                             odom.pose.pose.position.x - last_odom_->pose.pose.position.x);
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), angle));

    // assign latest_odom
    last_odom_ = std::make_shared<nav_msgs::msg::Odometry>(odom);    

    return last_odom_;
  }

  // bool LocalizationHack::isFarFromLastOdom(const nav_msgs::msg::Odometry *odom);
  // {
  //   float distance = std::sqrt(std::pow(odom->pose.pose.position.x - last_odom_->pose.pose.position.x, 2) + 
  //                              std::pow(odom->pose.pose.position.y - last_odom_->pose.pose.position.y, 2) + 
  //                              std::pow(odom->pose.pose.position.z - last_odom_->pose.pose.position.z, 2));
  //   return distance >= this->get_parameter("min_dist").as_double();
  // }

  void LocalizationHack::publishTransformFromOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
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

  void LocalizationHack::parse_datum()
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

  bool LocalizationHack::is_steering_angle_computable(const CartesianPosition cartesian_position)
  {
    // check if the last position is available
    if (this->last_odom_ == nullptr) {
      return false;
    }
    // check if the distance between the current position and the last position is greater than min_dist
    double dist = std::sqrt(std::pow(cartesian_position.x - this->last_odom_->pose.pose.position.x, 2) +
                            std::pow(cartesian_position.y - this->last_odom_->pose.pose.position.y, 2) +
                            std::pow(cartesian_position.z - this->last_odom_->pose.pose.position.z, 2));

    if (dist < this->get_parameter("min_dist").as_double())
    {
      return false;
    }

    return true;
  }

  void LocalizationHack::convert_gnss_to_local_cartesian(gps_msgs::msg::GPSFix::ConstSharedPtr input, CartesianPosition &outputCartesianPosition)
  {
    proj.Forward(input->latitude,
                 input->longitude,
                 input->altitude,
                 outputCartesianPosition.x,
                 outputCartesianPosition.y,
                 outputCartesianPosition.z);
  }
  float LocalizationHack::getYawFromOdom(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    tf2::Quaternion q(odom->pose.pose.orientation.x,
                      odom->pose.pose.orientation.y,
                      odom->pose.pose.orientation.z,
                      odom->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
  }
} // namespace roar

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<roar::LocalizationHack>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
