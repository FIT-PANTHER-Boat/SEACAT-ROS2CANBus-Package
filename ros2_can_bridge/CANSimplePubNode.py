#!/usr/bin/python3

import can
import rclpy
from rclpy.node import Node
from seacat_msg.msg import CANFrame

class CANSimplePubNode(Node):
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
	can_bridge_node = CANSimplePubNode()
	can_bridge_node.publish_can_data()	#Start publishing CAN data
	rclpy.spin(can_bridge_node)		#Spin the ROS2 node
	can_bridge_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()

