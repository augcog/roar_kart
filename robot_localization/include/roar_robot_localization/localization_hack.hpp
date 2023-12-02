#ifndef LOCALIZATION_HACK_
#define LOCALIZATION_HACK_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gps_msgs/msg/gps_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <iomanip> // Include the <iomanip> header for setprecision
#include <sstream>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;

namespace roar
{

  struct BufferData
  {
    sensor_msgs::msg::Imu::ConstSharedPtr imu_msg;
    gps_msgs::msg::GPSFix::ConstSharedPtr gps_msg;
    geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg;
  };

  struct CartesianPosition
  {
    double x;
    double y;
    double z;
  };
  struct GeodeticPosition
  {
    double latitude;
    double longitude;
    double altitude;
  };

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, gps_msgs::msg::GPSFix>
      ApproximateSyncPolicy;
  class LocalizationHack : public rclcpp::Node
  {
  public:
    LocalizationHack();
    ~LocalizationHack();

  protected:
    rclcpp::Subscription<gps_msgs::msg::GPSFix>::SharedPtr gps_sub_;
    void topic_callback(const gps_msgs::msg::GPSFix::SharedPtr gps_msg);
    // publisher
    void parse_datum();
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    void publishTransformFromOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    std::shared_ptr<nav_msgs::msg::Odometry> computeOdomFromGps(const gps_msgs::msg::GPSFix::SharedPtr gps_msg);
    std::unique_ptr<tf2_ros::TransformBroadcaster>
        tf_broadcaster_;

    // data
    GeographicLib::LocalCartesian proj;
    GeodeticPosition map_origin_;

    bool is_steering_angle_computable(const CartesianPosition cartesian_position);
    void convert_gnss_to_local_cartesian(gps_msgs::msg::GPSFix::ConstSharedPtr input, CartesianPosition &outputCartesianPosition);
    float getYawFromOdom(const nav_msgs::msg::Odometry::SharedPtr odom);
    std::shared_ptr<nav_msgs::msg::Odometry> last_odom_;
    // bool isFarFromLastOdom(const nav_msgs::msg::Odometry *odom);
  };

} // namespace roar

#endif // LOCALIZATION_HACK_