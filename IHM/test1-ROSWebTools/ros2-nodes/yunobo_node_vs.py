import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt64
from rclpy.time import Time
import socket
import struct
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from interfaces.msg import CommandTimestamp  # Replace with the actual package name

class YunoboNode(Node):
    def __init__(self):
        super().__init__('yunobo_node')

        # Define QoS with best-effort reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Setup socket
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('yunobo', 12345))  # Replace with actual RobotNode IP

        # Setup publishers
        self.time_stamp_publisher = self.create_publisher(CommandTimestamp, '/cmd_vel_time_stamp', qos_profile)
        self.temp_completion_publisher = self.create_publisher(UInt64, '/temp_completion', 1)

        # Subscribe to /cmd_vel topic
        self.create_subscription(TwistStamped, '/cmd_vel', self.listener_callback, qos_profile)

    def listener_callback(self, msg):
        # Get current time for T2
        T2 = self.get_current_time_in_ns()
        command_id = self.extract_ns_from_header(msg.header.stamp)

        # Send command to robot
        self.send_cmd_vel_to_robot(msg, command_id)

        # Publish timestamp
        self.publish_timestamp(command_id, 2, T2)

        # Wait for completion from robot
        self.receive_completion()

    def send_cmd_vel_to_robot(self, msg, command_id):
        # Pack command_id, linear and angular velocities
        message = struct.pack('!Qff', command_id, msg.twist.linear.x, msg.twist.angular.z)
        self.client_socket.sendall(message)

    def receive_completion(self):
        # Directly receive completion message from the robot
        data = self.client_socket.recv(16)  # Expecting 16 bytes (2 * 8 bytes)
        T4 = self.get_current_time_in_ns()
        command_id, T3 = struct.unpack('!QQ', data)  # Unpack command ID and T3

        # Publish the command completion message
        self.temp_completion_publisher.publish(UInt64(data=command_id))
        self.publish_timestamp(command_id, 3, T3)
        self.publish_timestamp(command_id, 4, T4)

    def publish_timestamp(self, command_id, timestamp_index, timestamp_value):
        """Create and publish timestamp message."""
        msg = CommandTimestamp()
        msg.command_id = command_id
        msg.timestamp_index = timestamp_index
        msg.timestamp = timestamp_value
        self.time_stamp_publisher.publish(msg)

    def get_current_time_in_ns(self):
        """Return current time in nanoseconds."""
        return self.get_clock().now().nanoseconds

    def extract_ns_from_header(self, stamp):
        """Extract timestamp in nanoseconds from a ROS message header stamp."""
        return Time.from_msg(stamp).nanoseconds

def main(args=None):
    rclpy.init(args=args)
    node = YunoboNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
