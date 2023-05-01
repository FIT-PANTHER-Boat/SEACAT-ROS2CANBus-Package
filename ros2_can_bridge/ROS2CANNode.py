#!/usr/bin/python3
# Ported to Python from the ros2can_bridge package by philippwuestenberg.
# https://github.com/ROS4SPACE/ros2can_bridge
#
# Last modified by: Braidan Duffy
# Last data modified: 04/21/2023

import can
import rclpy
from rclpy.node import Node
from seacat_msg.msg import CANFrame

class ROS2CANNode(Node):
    bus: can.interface.Bus

    def __init__(self):
        super().__init__('ROS2CANNode')

        self.declare_parameter('bus_name', 'can0')
		
        # Set up the CAN Interface
        self.bus_name = self.get_parameter('bus_name').get_parameter_value().string_value
        self.bus = can.interface.Bus(channel=self.bus_name, bustype='socketcan')
        self.get_logger().info(f"Using bus: {self.bus_name}")
        
        if not self.bus: # Check if bus successfully created
            self.get_logger().error("Bus already open!")
            raise RuntimeError(f"CAN Bus '{self.bus_name}' is already in use!")

        # Set up topic names
        self.topicname_receive = f"CAN/{self.bus_name}/receive"
        self.topicname_transmit = f"CAN/{self.bus_name}/transmit"

        # Set up the ROS2 publisher/receivers
        self.publisher_ = self.create_publisher(CANFrame, 
            self.topicname_receive, 
            10)
        self.test_publisher_ = self.create_publisher(CANFrame, 
            self.topicname_transmit, 
            10)
        self.subscription_ = self.create_subscription(
            CANFrame,
            self.topicname_transmit,
            self.CAN_send,
            10)

        self.get_logger().info(f"ROS2 to CAN bus topic: {self.topicname_transmit}")
        self.get_logger().info(f"CAN bus to ROS2 topic: {self.topicname_receive}")

    def CAN_send(self, msg: CANFrame):
        _frame = can.Message(arbitration_id=msg.id,
            is_extended_id=msg.eff,
            is_remote_frame=msg.rtr,
            is_error_frame=msg.err,
            dlc=msg.dlc,
            data=msg.data)
        self.bus.send(_frame)

        self.get_logger().info(f"Sent CAN message with ID: {_frame.arbitration_id} and Data: {_frame.data}")

    def CAN_listener(self):
        message = CANFrame()
        can_msg = self.bus.recv()	# Receive a CAN message 
        message.id = can_msg.arbitration_id	#Extract the CAN ID from the receieved message
        message.dlc = can_msg.dlc
        message.data = can_msg.data		#Extract the Can data from the received message
        self.get_logger().info(f'Received CAN message: ID={message.id}, Data={message.data}')

        self.publisher_.publish(message) # Publish the ROS2 message to the "can_data" topic

def main(args=None):
    rclpy.init(args=args)
    can_bridge_node = ROS2CANNode()
    can_bridge_node.CAN_listener()	#Start publishing CAN data
    rclpy.spin(can_bridge_node)		#Spin the ROS2 node
    can_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()