import can
import rclpy
from rclpy.node import Node
from seacat_msg.msg import CANFrame
from std_msgs.msg import Bool, Int16, Int32

class VESCCANNode(Node):
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
        "STATUS_5"                          : 16  # Periodic status of motor speed (tachometer), input voltage, and unknown
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
    
    
    # === SUBSCRIPTION HANDLERS ===
    
    
    def CAN_parse(self, msg: CANFrame):     
        if msg.id == self.VESC_MESSAGE_IDS["STATUS_1"]:
            self.get_logger().debug(f"Received VESC Status 1 update message with data: {msg.data}")
            self.publisher_motorspeed.publish(Int32(data=int.from_bytes(msg.data[0:3], "big")))
            self.publisher_current.publish(Int16(data=int.from_bytes(msg.data[4:5], "big")))
            self.publisher_dutycycle.publish(Int16(data=int.from_bytes(msg.data[6:7], "big")))
        else:
            return
    
    
    def handle_throttle(self, msg: Int16):
        self.get_logger().info(f"Received throttle command for {msg.data} us")
        _motor_dc = bytearray(int(self._map(msg.data, 1100, 1900, -1E5, 1E5)).to_bytes(4, "big", signed=True))
        self.get_logger().info(f"Convert to array: {_motor_dc}")
        _msg = CANFrame(id=self.VESC_MESSAGE_IDS["CMD_DUTY_CYCLE"],
                        dlc=len(_motor_dc),
                        data=_motor_dc)
        self.publisher_can.publish(_msg)
        
        
    # === UTILITY FUNCTIONS ===
    
    
    def _map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        

def main(args=None):
    rclpy.init(args=args)
    vesc_bridge_node = VESCCANNode()
    rclpy.spin(vesc_bridge_node)		#Spin the ROS2 node
    vesc_bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()