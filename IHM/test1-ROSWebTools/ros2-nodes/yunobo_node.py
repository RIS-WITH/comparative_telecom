import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt64MultiArray, UInt64
from rclpy.time import Time
import socket
import struct
import threading

class YunoboNode(Node):
    def __init__(self):
        super().__init__('yunobo_node')

        # Setup socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 12345))  # Bind to all interfaces
        self.server_socket.listen(1)  # Listen for one connection
        self.get_logger().info("YunoboNode: Waiting for RobotNode connection...")

        # Start a thread to accept client connections
        threading.Thread(target=self.accept_connections, daemon=True).start()

        # Subscriptions
        self.create_subscription(TwistStamped, '/cmd_vel', self.listener_callback, 1)

        # Publishers
        self.time_stamp_publisher = self.create_publisher(UInt64MultiArray, '/cmd_vel_time_stamp', 10)
        self.temp_completion_publisher = self.create_publisher(UInt64, '/temp_completion', 1)

        # Initialize client_socket
        self.client_socket = None

    def accept_connections(self):
        self.client_socket, client_address = self.server_socket.accept()
        self.get_logger().info(f"YunoboNode: Connected to {client_address}")

        while True:
            try:
                # Receive completion message from the robot
                data = self.receive_full_data(16)  # Expecting 16 bytes (2 * 8 bytes)
                if not data:
                    break  # Exit if the connection is closed
                self.process_completion(data)
            except Exception as e:
                self.get_logger().error(f"YunoboNode: Error receiving data: {e}")
                break

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
        T4 = self.get_current_time_in_ns()  # Get current time for T4
        command_id, T3 = struct.unpack('!QQ', data)  # Unpack command ID and timestamp

        self.temp_completion_publisher.publish(UInt64(data=command_id))  # Publish temporary completion message
        self.get_logger().info(f"YunoboNode: Command {command_id} completed with T3: {T3} and T4: {T4}")

        # Publish timestamp
        self.publish_timestamp(command_id, 3, T3)
        self.publish_timestamp(command_id, 4, T4)

    def listener_callback(self, msg):
        # Get current time for T2
        T2 = self.get_current_time_in_ns()

        # Extract command_id from message header stamp
        command_id = self.extract_ns_from_header(msg.header.stamp)

        self.get_logger().info(f'YunoboNode received cmd_vel: ({msg.twist.linear.x}, {msg.twist.angular.z})')

        # Send cmd_vel to the robot
        self.send_cmd_vel_to_robot(msg, command_id)
        self.get_logger().info(f'YunoboNode sent cmd_vel: ({msg.twist.linear.x}, {msg.twist.angular.z})')

        # Publish timestamp
        self.publish_timestamp(command_id, 2, T2)

    def send_cmd_vel_to_robot(self, msg, command_id):
        # Pack command_id
        header_message = struct.pack('!Q', command_id)
        self.client_socket.sendall(header_message)
        # Pack linear and angular velocities
        linear_x = msg.twist.linear.x
        angular_z = msg.twist.angular.z
        cmd_vel_message = struct.pack('!ff', linear_x, angular_z)  # Pack linear and angular velocities
        self.client_socket.sendall(cmd_vel_message)

    def publish_timestamp(self, command_id, timestamp_index, timestamp_value):
        # Create and publish timestamp message
        msg = UInt64MultiArray(data=[command_id, timestamp_index, timestamp_value])
        self.time_stamp_publisher.publish(msg)
        self.get_logger().info(f'Yunobo published timestamp: {msg.data}')

    def get_current_time_in_ns(self):
        return self.get_clock().now().nanoseconds
    
    def extract_ns_from_header(self, stamp):
        return Time.from_msg(stamp).nanoseconds

def main(args=None):
    rclpy.init(args=args)
    node = YunoboNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
