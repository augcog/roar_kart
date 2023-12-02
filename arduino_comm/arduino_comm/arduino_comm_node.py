# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header, Float32
from ackermann_msgs.msg import AckermannDriveStamped
from roar_gokart_msgs.msg import VehicleStatus, EgoVehicleControl, Actuation
from typing import Optional
import socket
import time
import struct
import json
from pydantic import BaseModel
from pydantic.types import confloat
from rclpy.logging import LoggingSeverity
import numpy as np


class ArduinoActuationModel(BaseModel):
    throttle: float = 0.0
    steering: float = 0.0
    brake: float = 0.0
    reverse: bool = False


class ArduinoVehicleStateModel(BaseModel):
    is_auto: bool = False
    is_left_limiter_ON: bool = False
    is_right_limiter_ON: bool = False
    angle: float = 0.0
    angular_velocity: float = 0.0
    speed: float = 0.0
    target_speed: float = 0.0
    target_steering_angle: float = 0.0
    current_actuation: ArduinoActuationModel = ArduinoActuationModel()


class ArduinoEgoVehicleControlMsg(BaseModel):
    target_speed: float = 0.0
    steering_angle: float = 0.0
    brake: confloat(ge=0.0, le=1.0) = 0.0
    reverse: bool = False


class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__("arduino_comm_node")
        self.declare_parameter("ip_address", "10.0.0.9")
        self.declare_parameter("port", 1883)

        self.declare_parameter("get_state_period", 0.1)
        self.declare_parameter("write_action_period", 0.1)

        self.declare_parameter("write_timeout", 0.1)
        self.declare_parameter("vehicle_status_header", "base_link")

        # timers
        get_state_period: float = (
            self.get_parameter("get_state_period").get_parameter_value().double_value
        )
        self.get_state_timer = self.create_timer(
            get_state_period, self.get_state_from_arduino
        )
        write_action_period: float = (
            self.get_parameter("write_action_period").get_parameter_value().double_value
        )
        self.write_action_timer = self.create_timer(
            write_action_period, self.write_action_callback
        )

        # pub / sub
        self.cmd_sub_ = self.create_subscription(
            EgoVehicleControl, "ego_vehicle_control", self.cmd_recvd, 10
        )
        self.vehicle_state_publisher_ = self.create_publisher(
            VehicleStatus, "vehicle_status", 10
        )

        self.state: Optional[VehicleStatus] = None
        self.latest_control_model: ArduinoEgoVehicleControlMsg = (
            ArduinoEgoVehicleControlMsg()
        )
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._logger.set_level(LoggingSeverity.INFO)

    def cmd_recvd(self, msg: EgoVehicleControl):
        self.latest_control_model = self.p_egoVehicleControlMsgToArduinoCmdActionModel(
            msg
        )
        self.get_logger().debug(f"Recevied: {self.latest_control_model}")

    def get_state_from_arduino(self):
        message = b"s"
        self.sock.sendto(message, self.p_getIPAndPort())

        data, addr = self.sock.recvfrom(1024)  # Buffer size is 1024 bytes
        try:
            string_data = data.decode("utf-8")
            json_data = json.loads(string_data)
            latest_model = self.p_dataToVehicleState(json_data)
            self.p_publish_state(latest_model)
            self.get_logger().debug(f"Read: [{latest_model}]")
        except Exception as e:
            self.get_logger().error(f"Unable to convert [{data}]. Error: {e}")

    def write_action_callback(self):
        message = self.latest_control_model.json().encode("utf-8")
        self.get_logger().debug(f"Wrote: {message}")
        self.sock.sendto(message, self.p_getIPAndPort())

    def p_egoVehicleControlMsgToArduinoCmdActionModel(
        self, egoVehicleControlMsg: EgoVehicleControl
    ) -> ArduinoEgoVehicleControlMsg:
        model = ArduinoEgoVehicleControlMsg()
        model.brake = np.clip(egoVehicleControlMsg.brake, 0, 1)
        model.reverse = egoVehicleControlMsg.reverse
        model.steering_angle = egoVehicleControlMsg.steering_angle
        model.target_speed = egoVehicleControlMsg.target_speed

        return model

    def p_getIPAndPort(self):
        ip_address = self.get_parameter("ip_address").get_parameter_value().string_value
        port = self.get_parameter("port").get_parameter_value().integer_value
        return (ip_address, port)

    def p_dataToVehicleState(self, data: dict) -> ArduinoVehicleStateModel:
        model = ArduinoVehicleStateModel.parse_obj(data)
        return model

    def p_publish_state(self, curr_state: ArduinoVehicleStateModel):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = (
            self.get_parameter("vehicle_status_header")
            .get_parameter_value()
            .string_value
        )
        act = Actuation()
        act.throttle = curr_state.current_actuation.throttle
        act.steering = curr_state.current_actuation.steering
        act.brake = curr_state.current_actuation.brake
        act.reverse = curr_state.current_actuation.reverse

        msg = VehicleStatus()

        msg.angle = curr_state.angle
        msg.is_auto = curr_state.is_auto
        msg.is_left_limiter_on = curr_state.is_left_limiter_ON
        msg.is_right_limiter_on = curr_state.is_right_limiter_ON
        msg.speed = curr_state.speed
        msg.target_speed = curr_state.target_speed
        msg.target_steering_angle = curr_state.target_steering_angle
        msg.actuation = act

        self.vehicle_state_publisher_.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()

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
