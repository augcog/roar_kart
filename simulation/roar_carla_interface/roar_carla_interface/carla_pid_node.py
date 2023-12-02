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
from roar_simulation_msgs.msg import VehicleControl
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
import math
from pydantic import BaseModel

# from tf2_ros import TransformListener
# from tf2_geometry_msgs import QuaternionStamped

from tf_transformations import euler_from_quaternion
import tf2_ros
from .pid import PID


class VehicleStatus(BaseModel):
    speed: float = 0.0
    heading: float = 0.0


class RoarCarlaPIDNode(Node):
    def __init__(self):
        super().__init__("roar_carla_pid_node")
        self.declare_parameter("debug", False)
        self.declare_parameter("throttle_kp", 1.0)
        self.declare_parameter("throttle_kd", 0.0)
        self.declare_parameter("throttle_ki", 0.0)
        self.declare_parameter("steering_kp", 1.0)
        self.declare_parameter("steering_kd", 0.0)
        self.declare_parameter("steering_ki", 0.0)
        self.declare_parameter("pid_loop_rate", 0.05)
        self.declare_parameter("max_steering_angle_deg", 30.0)

        self.cmd_sub_ = self.create_subscription(
            VehicleControl, "vehicle_control", self.on_cmd_received, 10
        )
        self.status_sub_ = self.create_subscription(
            CarlaEgoVehicleStatus, "vehicle_status", self.on_status_received, 10
        )
        self.cmd_pub_ = self.create_publisher(
            CarlaEgoVehicleControl, "carla_control", 10
        )

        self.pid_timer = self.create_timer(
            self.get_parameter("pid_loop_rate").get_parameter_value().double_value,
            self.on_pid_compute,
        )

        self.throttle_pid: PID = PID(
            Kp=self.get_parameter("throttle_kp").get_parameter_value().double_value,
            Kd=self.get_parameter("throttle_kd").get_parameter_value().double_value,
            Ki=self.get_parameter("throttle_ki").get_parameter_value().double_value,
            setpoint=0,
            output_limits=(0, 1),
        )
        self.steering_pid: PID = PID(
            Kp=self.get_parameter("steering_kp").get_parameter_value().double_value,
            Kd=self.get_parameter("steering_kd").get_parameter_value().double_value,
            Ki=self.get_parameter("steering_ki").get_parameter_value().double_value,
            setpoint=0,
            output_limits=(-1, 1),
        )

        # misc
        self.latest_vehicle_state: Optional[CarlaEgoVehicleStatus] = None
        self.latest_control_msg: Optional[VehicleControl] = None

        if self.get_parameter("debug").get_parameter_value().bool_value:
            self.get_logger().set_level(LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info(f"PID Node initialized")

    def on_pid_compute(self):
        if self.latest_vehicle_state and self.latest_control_msg:
            try:
                current_velocity = self.latest_vehicle_state.velocity
                target_speed = self.latest_control_msg.target_speed
                self.throttle_pid.setpoint = target_speed

                throttle = self.throttle_pid(current_velocity)

                control = CarlaEgoVehicleControl()
                control.header = self.latest_control_msg.header

                control.throttle = float(throttle)
                control.brake = float(self.latest_control_msg.brake)
                control.reverse = bool(self.latest_control_msg.reverse)

                max_steering_angle_deg = (
                    self.get_parameter("max_steering_angle_deg")
                    .get_parameter_value()
                    .double_value
                )
                mapped_value = self.map_value(
                    self.latest_control_msg.steering_angle,
                    (-max_steering_angle_deg, max_steering_angle_deg),
                    (-1, 1),
                )

                control.steer = float(mapped_value)

                self.cmd_pub_.publish(control)
            except Exception as e:
                self._logger.info(f"ERROR: {e}")

    def on_cmd_received(self, msg: VehicleControl):
        max_steering_angle_deg = (
            self.get_parameter("max_steering_angle_deg")
            .get_parameter_value()
            .double_value
        )
        msg.steering_angle = self.clip_value(
            msg.steering_angle, -max_steering_angle_deg, max_steering_angle_deg
        )
        self.latest_control_msg = msg

    def on_status_received(self, msg: CarlaEgoVehicleStatus):
        self.latest_vehicle_state = msg

    def convert_orientation_to_heading(self, orientation):
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        yaw = math.degrees(yaw)
        return yaw

    @staticmethod
    def map_value(value, input_range, target_range):
        input_min, input_max = input_range
        target_min, target_max = target_range

        mapped_value = (value - input_min) * (target_max - target_min) / (
            input_max - input_min
        ) + target_min

        return mapped_value

    @staticmethod
    def clip_value(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoarCarlaPIDNode()

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
