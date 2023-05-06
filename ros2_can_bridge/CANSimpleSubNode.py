#!/usr/bin/python3
"""
Simple example of a subscriber node that receives a CAN bus frame from a ROS topic

Date Created: 05/01/2023
Date Modified: 05/06/2023

CHANGELOG:
v1.0.0 - Initial Release
v1.0.1 - Updated docs
"""

import rclpy
from rclpy.node import Node

from seacat_msg.msg import CANFrame

__author__ = "Braidan Duffy"
__copyright__ = "Copyright 2023, PANTHER Boat Team"
__credits__ = ["Braidan Duffy"]
__license__ = "MIT"
__version__ = "1.0.1"
__maintainer__ = "Braidan Duffy"
__email__ = "bduffy2018@my.fit.edu"
__status__ = "Example"


class CANSimpleSubNode(Node):
    """
    A simple node that subscribes to a ROS topic for CAN frames
    """
    
    def __init__(self):
        super().__init__('can_simple_sub_node')
        self.subscription = self.create_subscription(
            CANFrame,
            'CAN/can0/receive',
            self.listener_callback,
            10)

    def listener_callback(self, msg: CANFrame):
        """
        Listens for a CAN frame from the ROS topic and reports it to the logger

        Args:
            msg (CANFrame): A CAN frame sent from another node in the network
        """
        self.get_logger().info('Received CAN message: ID={}, Data={}'.format(msg.id, msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = CANSimpleSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
