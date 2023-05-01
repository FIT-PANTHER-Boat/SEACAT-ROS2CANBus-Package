# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from seacat_msg.msg import CANFrame


class CANSimpleSubNode(Node):

    def __init__(self):
        super().__init__('can_simple_sub_node')
        self.subscription = self.create_subscription(
            CANFrame,
            'CAN/can0/receive',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received CAN message: ID={}, Data={}'.format(msg.id, msg.data))


def main(args=None):
    rclpy.init(args=args)

    simple_sub_node = CANSimpleSubNode()

    rclpy.spin(simple_sub_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_sub_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
