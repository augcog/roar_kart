# Copyright 2023 michael. All rights reserved.
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge
import cv2

class CarlaDepthImageConverter(Node):
    def __init__(self):
        super().__init__("carla_depth_img_converter_node")

        # timers
        self.declare_parameter("refresh_rate", 0.05)
        self.declare_parameter("debug", False)
        self.declare_parameter("speed_increment", 0.1)
        self.declare_parameter("angle_increment", 1.0)
        self.declare_parameter("frame_id", "ego_vehicle")

        # pub / sub
        self.cmd_publisher = self.create_publisher(Image, "carla_image_processed", 10)

        self.rgb_sub = self.create_subscription(
            Image, "carla_depth", self.on_image_received, 10
        )
        self.cv_bridge_ = CvBridge()
        self.scales = np.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000

    def on_image_received(self, msg: Image):
        try:
            image = self.cv_bridge_.imgmsg_to_cv2(msg)
            image = image / image.max() * 255.0
            output = msg 
            output.data = image.tobytes()
            self.cmd_publisher.publish(output)
        except Exception as e:
            self.get_logger().error(f"{e}")



def main(args=None):
    rclpy.init(args=args)
    node = CarlaDepthImageConverter()

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
