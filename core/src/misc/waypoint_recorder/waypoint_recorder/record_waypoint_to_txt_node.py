# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from pathlib import Path
from nav_msgs.msg import Odometry

class ROARWaypointToTxtNode(Node):
    def __init__(self):
        super().__init__("roar_waypoint_to_txt_node")
        self.declare_parameter("recording_path", "./data")
        sub = self.create_subscription(Odometry, 
                                       "/roar/odometry", 
                                       self.odom_received, 
                                       10)

        self.recording_dir = Path(self.get_parameter("recording_path").get_parameter_value().string_value)
        self.get_logger().info(f"Recording path: {self.recording_dir}")
        self.recording_dir.mkdir(exist_ok=True, parents=True)
        self.recording_path = self.recording_dir / "waypoint.txt"
        self.recording_file = self.recording_path.open("w")
        self.recording_file.write("timestamp,frame_id,child_frame_id,x,y,z,qx,qy,qz,qw,vx,vy,vz,rx,ry,rz\n")
        
        self.get_logger().info(f"ROARWaypointToTxtNode has started. Listening topic: {sub.topic}")

    def odom_received(self, msg:Odometry):
        msg = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec},{msg.header.frame_id},{msg.child_frame_id},{msg.pose.pose.position.x},{msg.pose.pose.position.y},{msg.pose.pose.position.z},{msg.pose.pose.orientation.x},{msg.pose.pose.orientation.y},{msg.pose.pose.orientation.z},{msg.pose.pose.orientation.w},{msg.twist.twist.linear.x},{msg.twist.twist.linear.y},{msg.twist.twist.linear.z},{msg.twist.twist.angular.x},{msg.twist.twist.angular.y},{msg.twist.twist.angular.z}\n"
        self.recording_file.write(msg)

    def destroy_node(self):
        self.recording_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ROARWaypointToTxtNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
