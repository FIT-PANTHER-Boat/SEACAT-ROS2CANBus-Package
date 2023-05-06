#!/usr/bin/python3
"""
Simple example of a publisher node that receives a CAN bus frame and directly publishes it to a ROS topic

Date Created: 05/01/2023
Date Modified: 05/06/2023

CHANGELOG:
v1.0.0 - Initial Release
v1.0.1 - Updated docs
"""

import can
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


class CANSimplePubNode(Node):
	"""
 	A simple node for publishes received CAN messages directly to a ROS topic
	"""
 
	def __init__(self, can_intr: str):
		super().__init__('can_simple_pub_node')

		self.get_logger().info("Using bus: %s", can_intr)
		
		# Set up the CAN Interface
		self.can_interface = can_intr
		self.bus = can.interface.Bus(channel=can_intr, bustype='socketcan')

		# Set up the ROS2 publisher
		self.publisher_ = self.create_publisher(CANFrame, 
			'CAN/can0/transmit', 
			10)

	def publish_can_data(self):
		"""Constantly listen for CAN messages and send them to a ROS topic
		
  		Note: This is **NOT** the proper way to execute this command.
		This function should be assigned to a timer ('self.create_timer(PERIOD_S, self.publish_can_data)') where PERIOD_S is how often the function is called.
		The 'while True' method blocks all other functions in the node.
		"""
		while True:
			message = CANFrame()
			can_msg = self.bus.recv()	# Receive a CAN message 
			message.id = can_msg.arbitration_id	#Extract the CAN ID from the receieved message
			message.dlc = can_msg.dlc
			message.data = can_msg.data		#Extract the Can data from the received message
			self.get_logger().info('Received CAN message: ID={}, Data={}'.format(message.id, message.data))

			self.publisher_.publish(message) # Publish the ROS2 message to the "can_data" topic


def main (args=None):
	rclpy.init(args=args)
	node = CANSimplePubNode()
	node.publish_can_data()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

