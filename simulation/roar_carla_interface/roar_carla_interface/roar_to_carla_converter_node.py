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
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
import math
from pydantic import BaseModel
from roar_simulation_msgs.msg import VehicleControl as SimVehicleControl
from roar_msgs.msg import VehicleControl as RoarVehicleControl

from tf_transformations import euler_from_quaternion
import tf2_ros
from .pid import PID


class RoarCarlaConverterNode(Node):
    def __init__(self):
        super().__init__("roar_carla_converter_node")
        self.latest_control_msg: Optional[RoarVehicleControl] = None
        self.publisher = self.create_publisher(SimVehicleControl, "sim_vehicle_control", 10)
        self.subscription = self.create_subscription(RoarVehicleControl, "roar_vehicle_control", self.on_cmd_received, 10)
    def on_cmd_received(self, msg: RoarVehicleControl):
        output = SimVehicleControl()
        output.header = msg.header
        output.target_speed = msg.target_speed
        output.steering_angle = msg.steering_angle
        output.brake = msg.brake
        output.reverse = msg.reverse
        self.publisher.publish(output)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoarCarlaConverterNode()

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
