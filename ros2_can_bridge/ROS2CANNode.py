#!/usr/bin/python3
"""
A node that bridges ROS topics to a physical CANbus interface through the socketcan interface.

Ported to Python from the ros2can_bridge package by philippwuestenberg.
https://github.com/ROS4SPACE/ros2can_bridge

Date created: 05/01/2023
Date modified: 05/05/2023

CHANGELOG:
v1.0.0 - Initial Release
v1.0.1 - Changed some of the parameter calls and variable names
v1.0.2 - Updated docs

TODO:
- Add exception handling for KeyboardInterrupt (close CAN bus)
- Add exception handling for can.exceptions.CanOperationError: Failed to transmit: No buffer space available (stop transmitting until bus opens up)
"""

import can
import rclpy
from rclpy.node import Node
from seacat_msg.msg import CANFrame

__author__ = "Braidan Duffy"
__copyright__ = "Copyright 2023, PANTHER Boat Team"
__credits__ = ["Braidan Duffy"]
__license__ = "GPL"
__version__ = "1.0.2"
__maintainer__ = "Braidan Duffy"
__email__ = "bduffy2018@my.fit.edu"
__status__ = "Production"

class ROS2CANNode(Node):
    """
    Node that bridges software ROS topics to the physical world via a physical CAN bus and the socketcan interface
    
    Parameters:
        bus_name (str): the CAN bus interface the VESC is connected. Default: 'can0'
    """
    BUS_NAME: str
    bus: can.interface.Bus

    def __init__(self):
        super().__init__('ROS2CANNode')

        self.declare_parameter('bus_name', 'can0')
		
        # Set up the CAN Interface
        self.BUS_NAME = self.get_parameter('bus_name').get_parameter_value().string_value
        self.bus = can.interface.Bus(channel=self.BUS_NAME, bustype='socketcan')
        self.get_logger().info(f"Using bus: {self.BUS_NAME}")
        
        if not self.bus: # Check if bus successfully created
            self.get_logger().fatal("Bus already open!")
            raise RuntimeError(f"CAN Bus '{self.BUS_NAME}' is already in use!")

        # Set up topic names
        self.topicname_receive = f"CAN/{self.BUS_NAME}/receive"
        self.topicname_transmit = f"CAN/{self.BUS_NAME}/transmit"

        # Set up the ROS2 publisher/subscribers
        self.publisher_ = self.create_publisher(CANFrame, 
                                                self.topicname_receive, 
                                                10)
        self.get_logger().info(f"Publishing received CAN frames to: {self.topicname_transmit}")
                
        self.subscription_ = self.create_subscription(CANFrame, 
                                                      self.topicname_transmit, 
                                                      self.CAN_send, 
                                                      10)
        self.get_logger().info(f"Listening for CAN frames at: {self.topicname_receive}")
        
        # Create a timer to call the listener function at 10 Hz
        self.timer_listener = self.create_timer(0.1, self.CAN_listener)
        self.get_logger().info("Created a timer to check for new CAN bus messages at 10 Hz")


    # =============================
    # === SUBSCRIPTION HANDLERS ===
    # =============================


    def CAN_send(self, msg: CANFrame):
        """
        Formats a CAN bus message from a ROS CAN frame and sends it to the bus through the socketcan interface

        Args:
            msg (CANFrame): A CAN frame sent from within the ROS network
        """
        self.bus.send(can.Message(arbitration_id=msg.id, 
                                  is_extended_id=msg.eff, 
                                  is_remote_frame=msg.rtr, 
                                  is_error_frame=msg.err, 
                                  dlc=msg.dlc, 
                                  data=msg.data))

        self.get_logger().info(f"Sent CAN message with ID: {msg.id} and Data: {msg.data}")

    def CAN_listener(self):
        """
        Listens for a message from the CAN bus and then publishes it to a ROS topic
        """
        can_msg = self.bus.recv()	                            # Receive a CAN message 
        message = CANFrame(id=can_msg.arbitration_id,	        # Extract the CAN ID from the receieved message
                           dlc=can_msg.dlc,                     # Extract the data length
                           data=can_msg.data,		            # Extract the data from the received message
                           err=can_msg.error_state_indicator,   # Extract if the frame is an ERROR f
                           rtr=can_msg.is_remote_frame,         # Extract if the frame is a REQUEST frame
                           eff=can_msg.is_extended_id)          # Extract if the frame uses an extended id
        self.get_logger().info(f'Received CAN message: ID={message.id}, Data={message.data}')

        self.publisher_.publish(message)                        # Publish the ROS2 message to the "can_data" topic


def main(args=None):
    rclpy.init(args=args)
    node = ROS2CANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()