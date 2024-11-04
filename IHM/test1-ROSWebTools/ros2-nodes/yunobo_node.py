import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt64
from rclpy.time import Time
import socket
import struct
import threading
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from interfaces.msg import CommandTimestamp  # Replace 'your_package_name' with the actual package name

class YunoboNode(Node):
    def __init__(self):
        super().__init__('yunobo_node')

        # Define minimal QoS configuration for low-latency, best-effort communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Setup and configure socket for client connection
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 12345))
        self.server_socket.listen(1)  # Limit to single connection

        # Start the connection thread
        threading.Thread(target=self.accept_and_receive, daemon=True).start()

        # Setup subscriptions and publishers
        self.create_subscription(TwistStamped, '/cmd_vel', self.listener_callback, qos_profile)
        self.time_stamp_publisher = self.create_publisher(CommandTimestamp, '/cmd_vel_time_stamp', qos_profile)
        self.temp_completion_publisher = self.create_publisher(UInt64, '/temp_completion', 1)

        # Initialize the client socket to None
        self.client_socket = None

    def accept_and_receive(self):
        """Accept a client connection and receive data."""
        self.client_socket, _ = self.server_socket.accept()
        try:
            while True:
                data = self.receive_full_data(16)  # Expecting 16 bytes (2 * 8 bytes)
                if not data:
                    break
                self.process_completion(data)
        except Exception as e:
            self.get_logger().error(f"YunoboNode: Connection error: {e}")
        finally:
            self.client_socket.close()
            self.server_socket.close()

    def receive_full_data(self, expected_bytes):
        """Receive data from the socket until the expected number of bytes is received."""
        data = bytearray()
        while len(data) < expected_bytes:
            packet = self.client_socket.recv(expected_bytes - len(data))
            if not packet:
                raise ConnectionError("Connection closed by remote host")
            data.extend(packet)
        return bytes(data)

    def process_completion(self, data):
        """Process completion message with command ID and timestamps."""
        T4 = self.get_current_time_in_ns()
        command_id, T3 = struct.unpack('!QQ', data)
        
        # Publish completion and timestamp messages
        self.temp_completion_publisher.publish(UInt64(data=command_id))
        self.publish_timestamp(command_id, 3, T3)
        self.publish_timestamp(command_id, 4, T4)

    def listener_callback(self, msg):
        """Handle incoming TwistStamped messages and send to robot."""
        T2 = self.get_current_time_in_ns()
        command_id = self.extract_ns_from_header(msg.header.stamp)
        
        # Send command to the robot
        self.send_cmd_vel_to_robot(msg, command_id)
        self.publish_timestamp(command_id, 2, T2)

    def send_cmd_vel_to_robot(self, msg, command_id):
        """Send command ID and velocity data to the robot."""
        # Send command ID and velocities as a single packed message
        message = struct.pack('!Qff', command_id, msg.twist.linear.x, msg.twist.angular.z)
        self.client_socket.sendall(message)

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
