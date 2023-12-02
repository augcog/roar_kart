# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import rclpy
from rclpy.node import Node
from pygame import *
import pygame
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from roar_msgs.msg import VehicleControl

from typing import Optional
from rclpy.logging import LoggingSeverity
import numpy as np
from cv_bridge import CvBridge
import cv2

from pydantic import BaseModel


class State(BaseModel):
    target_speed: float = 0.0
    steering_angle: float = 0.0
    brake: float = 0.0
    reverse: bool = False

    def __repr__(self) -> str:
        return f"target_spd: {self.target_speed:.3f} steering_angle: {self.steering_angle:.3f} brake: {self.brake:.3f} reverse: {self.reverse}"


class ManualControllerNode(Node):
    def __init__(self):
        super().__init__("roar_carla_manual_controller_node")

        # timers
        self.declare_parameter("refresh_rate", 0.05)
        self.declare_parameter("debug", False)
        self.declare_parameter("speed_increment", 0.1)
        self.declare_parameter("angle_increment", 1.0)
        self.declare_parameter("frame_id", "ego_vehicle")
        self.timer_callback = self.create_timer(
            self.get_parameter("refresh_rate").get_parameter_value().double_value,
            self.on_timer_callback,
        )

        # pub / sub
        self.cmd_publisher = self.create_publisher(VehicleControl, "roar/vehicle/control", 10)

        self.rgb_sub = self.create_subscription(
            Image, "image", self.on_image_received, 10
        )
        self.cv_bridge_ = CvBridge()
        self.image: Optional[np.ndarray] = None

        # pygame
        pygame.init()
        self.display = None
        self.surface = None
        pygame.display.set_caption(self.get_name())
        self.font = pygame.font.Font(None, 36)  # Choose the font and size

        # misc
        self.state = State()
        self.speed_inc = (
            self.get_parameter("speed_increment").get_parameter_value().double_value
        )
        self.angle_inc = (
            self.get_parameter("angle_increment").get_parameter_value().double_value
        )

        if self.get_parameter("debug").get_parameter_value().bool_value:
            self.get_logger().set_level(LoggingSeverity.DEBUG)
        else:
            self.get_logger().set_level(LoggingSeverity.INFO)

    def on_image_received(self, msg: Image):
        try:
            image = cv2.rotate(
                self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8"),
                cv2.ROTATE_90_COUNTERCLOCKWISE,
            )

            self.image = cv2.flip(image, 0)
        except Exception as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))
            return

    def on_timer_callback(self):
        if self.image is not None:
            if self.surface is None or self.display == None:
                # first render
                self.display = pygame.display.set_mode(
                    (self.image.shape[0], self.image.shape[1]),
                    pygame.HWSURFACE | pygame.DOUBLEBUF,
                )
                self.surface = pygame.surfarray.make_surface(self.image)
            frame: np.ndarray = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            pygame_image = pygame.surfarray.make_surface(frame)
            self.display.blit(pygame_image, (0, 0))

            text = self.font.render(
                self.state.__repr__(), True, (255, 0, 0)
            )  # Render the state as text
            self.display.blit(text, (10, 10))  # Blit the text onto the display surface
            pygame.display.flip()

        self.p_parse_event()
        self.p_publish(self.state)

    def p_publish(self, state: State):
        header: Header = Header()
        header.frame_id = (
            self.get_parameter("frame_id").get_parameter_value().string_value
        )

        header.stamp = self.get_clock().now().to_msg()

        control: VehicleControl = VehicleControl()
        control.header = header
        control.target_speed = float(state.target_speed)
        control.steering_angle = float(state.steering_angle)
        control.brake = float(state.brake)
        control.reverse = bool(state.reverse)
        self.cmd_publisher.publish(control)

    def p_parse_event(self):
        for event in pygame.event.get():
            if event.type == KEYDOWN:
                if event.key == K_r:
                    self.state.reverse = not self.state.reverse

                if event.key == K_w or event.key == K_UP:
                    self.state.target_speed = self.state.target_speed + self.speed_inc
                if event.key == K_s or event.key == K_DOWN:
                    self.state.target_speed = self.state.target_speed - self.speed_inc

                # if event.key == K_d or event.key == K_RIGHT:
                #     self.state.steering_angle = (
                #         self.state.steering_angle + self.angle_inc
                #     )
                # if event.key == K_a or event.key == K_LEFT:
                #     self.state.steering_angle = (
                #         self.state.steering_angle - self.angle_inc
                #     )

                if event.key == K_SPACE:
                    self.state.brake = 1 - self.state.brake
        keys = pygame.key.get_pressed()

        if keys[K_a] or keys[K_LEFT]:
            self.state.steering_angle = self.state.steering_angle - self.angle_inc
        if keys[K_d] or keys[K_RIGHT]:
            self.state.steering_angle = self.state.steering_angle + self.angle_inc

        self.get_logger().debug(f"Control: {self.state.__repr__()}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ManualControllerNode()

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
