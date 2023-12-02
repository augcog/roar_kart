# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import rclpy
from rclpy.node import Node
from pygame import *
import pygame
from roar_simulation_msgs.msg import VehicleControl
from sensor_msgs.msg import Image

from typing import Optional
from rclpy.logging import LoggingSeverity
import numpy as np
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import Float32
from roar_msgs.msg import VehicleControl
from roar_msgs.msg import VehicleStatus


class RoarCarlaStatePublisher(Node):
    def __init__(self):
        super().__init__("roar_carla_state_publisher")
        self.publisher = self.create_publisher(VehicleStatus, "vehicle_status", 10)
        self.vehicle_control_sub_ = self.create_subscription(
            VehicleControl, "vehicle_control", self.on_cmd_received, 10
        )
        self.vehicle_speed_sub_ = self.create_subscription(
            Float32, "vehicle_speed", self.on_speed_received, 10
        )
        self.timer = self.create_timer(0.05, self.on_timer)


        self.latest_steering_angle = 0.0
        self.latest_speed_reading = 0.0
    
    def on_timer(self):
        msg = VehicleStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.speed = float(self.latest_speed_reading)
        msg.steering_angle_deg = float(self.latest_steering_angle)
        self.publisher.publish(msg)

    def on_cmd_received(self, msg: VehicleControl):
        self.latest_steering_angle = msg.steering_angle

    def on_speed_received(self, msg: Float32):
        self.latest_speed_reading = msg.data

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoarCarlaStatePublisher()

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
