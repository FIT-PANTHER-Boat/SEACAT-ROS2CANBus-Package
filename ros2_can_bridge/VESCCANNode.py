#!/usr/bin/python3
"""
A node that handles calculations and CAN interractions with the VESC version 6.X

Date created: 05/03/2023
Date modified: 05/06/2023

CHANGELOG:
v1.0.0 - Initial Release
v1.0.1 - Updated docs

TODO:
- Add exception handling for KeyboardInterrupt
- Add support for sending VESC status messages to ROS topics
- Add support for custom ROS messages (e.g. Throttle, VESCStatus, etc.)
"""

import rclpy
from rclpy.node import Node
from seacat_msg.msg import CANFrame, Throttle
from std_msgs.msg import Bool, Int16, Int32

__author__ = "Braidan Duffy"
__copyright__ = "Copyright 2023, PANTHER Boat Team"
__credits__ = ["Braidan Duffy"]
__license__ = "GPL"
__version__ = "1.0.1"
__maintainer__ = "Braidan Duffy"
__email__ = "bduffy2018@my.fit.edu"
__status__ = "Production"

class VESCCANNode(Node):
    """
    A node that acts as the software representation of the VESC within the ROS network.
    Handles appropriate calculations and CAN bus messages to drive the VESC V6.X
    
    Parameters:
        bus_name (str): the CAN bus interface the VESC is connected. Default: 'can0'
        vesc_id (int): the CAN ID of the VESC (set by the VESC tool). Default: 126
        side (str): which side of SEACAT the VESC is on. Default: 'port'
    """
    # Message IDs (Note: Must bitshift left 8 and add 'vesc_id' for valid message ID)
    # Refer to the SEACAT CAN ICD for additional details
    VESC_MESSAGE_PREFIXES = {
        "CMD_DUTY_CYCLE"                    : 0,  # Set motor duty cycle 
        "CMD_SET_CURRENT"                   : 1,  # Set motor to current draw
        "CMD_SET_CURRENT_BRAKE"             : 2,  # Set amount VESC can regen during braking
        "CMD_SET_RPM"                       : 3,  # Set motor RPM
        "CMD_SET_POS"                       : 4,  # Set motor to certain position
        "STATUS_1"                          : 9,  # Periodic status of motor speed, current, and duty cycle
        "CMD_SET_RELATIVE_CURRENT"          : 10, # Set motor speed relative to current limits
        "CMD_SET_RELATIVE_BRAKE_CURRENT"    : 11, # Set motor braking current relative to limits
        "STATUS_2"                          : 14, # Periodic status of Amp-hours and Amp-hours charged (regened)
        "STATUS_3"                          : 15, # Periodic status of Watt-hours and Watt-hours charged (regened)
        "STATUS_4"                          : 16, # Periodic status of MOSFET temp, motor temp, total input current, and current PID position
        "CMD_SET_CURRENT_LIMIT"             : 21, # Set the output current limit
        "CMD_STORE_CURRENT_LIMIT"           : 22, # Store the output current limit to memory
        "CMD_SET_INPUT_CURRENT_LIMIT"       : 23, # Set the input current limit
        "CMD_STORE_INPUT_CURRENT_LIMIT"     : 24, # Store the input current limit to memory
        "STATUS_5"                          : 27  # Periodic status of motor speed (tachometer), input voltage, and unknown
    }
    VESC_MESSAGE_IDS: dict
    BUS_NAME: str
    SIDE_NAME: str
    CAN_TOPIC_PREFIX: str

    def __init__(self):
        super().__init__('VESCCANNode')

        self.declare_parameter('bus_name', 'can0')
        self.declare_parameter('vesc_id', 126)
        self.declare_parameter('side', 'port')
        
        # Set CAN bus name
        self.BUS_NAME = self.get_parameter('bus_name').get_parameter_value().string_value
        self.get_logger().info(f"Using bus: {self.BUS_NAME}")
        self.CAN_TOPIC_PREFIX = f"CAN/{self.BUS_NAME}"
        
        # Update the VESC message IDs
        self.VESC_ID = self.get_parameter('vesc_id').get_parameter_value().integer_value
        self.get_logger().info(f"Using device ID: {self.VESC_ID}")
        self.VESC_MESSAGE_IDS = {key: (val<<8)+self.VESC_ID for key, val in self.VESC_MESSAGE_PREFIXES.items()}
        self.get_logger().debug(f"Using VESC Messages: {self.VESC_MESSAGE_IDS}")
        
        # Set side name
        self.SIDE_NAME = self.get_parameter('side').get_parameter_value().string_value
        self.get_logger().info(f"Setting ESC side to: {self.SIDE_NAME}")

        # Set up topic names
        self.topicname_transmit     = f"{self.CAN_TOPIC_PREFIX}/transmit"
        self.topicname_receive      = f"{self.CAN_TOPIC_PREFIX}/receive"
        self.topicname_throttle     = f"throttle/{self.SIDE_NAME}"
        self.topicname_motorspeed   = f"motor_speed/{self.SIDE_NAME}"
        self.topicname_current      = f"motor_current/{self.SIDE_NAME}"
        self.topicname_dutycycle    = f"duty_cycle/{self.SIDE_NAME}"

        # Set up the ROS2 publisher/subscribers
        self.publisher_can = self.create_publisher(CANFrame,
                                                   self.topicname_transmit,
                                                   10)
        self.publisher_motorspeed = self.create_publisher(Int32,
                                                          self.topicname_motorspeed,
                                                          10)
        self.publisher_current = self.create_publisher(Int16,
                                                       self.topicname_current,
                                                       10)
        self.publisher_dutycycle = self.create_publisher(Int16,
                                                         self.topicname_dutycycle,
                                                         10)
        self.get_logger().info(f"Publishing CAN frames to: {self.topicname_transmit}")
        self.get_logger().info(f"Publishing motorspeed data to: {self.topicname_motorspeed}")
        self.get_logger().info(f"Publishing current draw data to: {self.topicname_current}")
        self.get_logger().info(f"Publishing motor duty cycle data to: {self.topicname_dutycycle}")
        
        self.subscription_can = self.create_subscription(CANFrame,
                                                         self.topicname_receive,
                                                         self.CAN_parse,
                                                         10)
        self.subscription_throttle = self.create_subscription(Int16,
                                                              self.topicname_throttle,
                                                              self.handle_throttle,
                                                              10)
        self.get_logger().info(f"Listening for CAN messages from: {self.topicname_receive}")
        self.get_logger().info(f"Listening for throttle commands from: {self.topicname_throttle}")
    
    
    # =============================
    # === SUBSCRIPTION HANDLERS ===
    # =============================
    
    
    def CAN_parse(self, msg: CANFrame):
        """
        Parses certain messages that are reported to the CAN bus receive ROS topic and reports appropriate data to other ROS topics.
        Typically, these will be status messages from the VESC (see SEACAT CAN ICD for more information).
        
        Args:
            msg (CANFrame): A CAN frame published to the receive ROS topic by the CAN bridge
        """
        if msg.id == self.VESC_MESSAGE_IDS["STATUS_1"]: # Check if message is a VESC STATUS_1
            self.get_logger().debug(f"Received VESC Status 1 update message with data: {msg.data}")
            self.publisher_motorspeed.publish(Int32(data=int.from_bytes(msg.data[0:3], "big")))
            self.publisher_current.publish(Int16(data=int.from_bytes(msg.data[4:5], "big")))
            self.publisher_dutycycle.publish(Int16(data=int.from_bytes(msg.data[6:7], "big")))
        else: # If message does not match anything desired, ignore
            return
    
    def handle_throttle(self, msg: Int16):
        """
        Receives a throttle message from the ROS topic and translates it to an appropriate value for the VESC.
        Note: typically, this throttle command will be in microseconds of PWM signal.
        1100 us - Full reverse, 1500 us - Netural, 1900 - Full forward.
        The VESC expects a duty cycle command of [-1E6, 1E6] where negative is reverse and positive is forward

        Args:
            msg (Int16): the throttle message from the ROS topic containing the PWM information
        """
        self.get_logger().debug(f"Received throttle command for {msg.data} us")
        _motor_dc = bytearray(int(self._map(msg.data, 1100, 1900, -1E5, 1E5)).to_bytes(4, "big", signed=True))
        self.get_logger().debug(f"Convert to array: {_motor_dc}")
        _msg = CANFrame(id=self.VESC_MESSAGE_IDS["CMD_DUTY_CYCLE"],
                        dlc=len(_motor_dc),
                        data=_motor_dc,
                        eff=True)
        self.publisher_can.publish(_msg)
        

    # =========================
    # === UTILITY FUNCTIONS ===
    # =========================
    
    
    def _map(self, x, in_min, in_max, out_min, out_max):
        """
        Maps a value from one number range to another.
        Does not perform any rounding

        Args:
            x: the intial value to be converted
            in_min: the minimal value of the input range
            in_max: the maximum value of the input range
            out_min: the minimum value of the output range
            out_max: the maximum value of the output range

        Returns:
            num: the converted initial value in the new range
        """
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        

def main(args=None):
    rclpy.init(args=args)
    node = VESCCANNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()