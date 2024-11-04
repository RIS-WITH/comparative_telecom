import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt64
import socket
import struct
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Define QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Setup socket
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind(('0.0.0.0', 12345))  # Bind to all interfaces
        self.server_socket.listen(1)  # Listen for one connection

        self.client_socket, _ = self.server_socket.accept()  # Accept a connection
        self.get_logger().info("RobotNode: Connected to YunoboNode")

        # Start listening for commands
        self.listen_for_commands()

    def listen_for_commands(self):
        while True:
            data = self.client_socket.recv(16)  # Expecting 16 bytes (8 for command_id, 8 for velocities)
            if not data:
                break  # Exit if the connection is closed
            self.process_command(data)

    def process_command(self, data):
        # Unpack the received command
        command_id, linear_x, angular_z = struct.unpack('!Qff', data)
        # Execute command (simulated)
        self.get_logger().info(f"RobotNode: Executing command {command_id} with velocities: linear_x={linear_x}, angular_z={angular_z}")

        # Send back completion message
        T3 = self.get_current_time_in_ns()
        completion_data = struct.pack('!QQ', command_id, T3)
        self.client_socket.sendall(completion_data)

    def get_current_time_in_ns(self):
        """Return current time in nanoseconds."""
        return self.get_clock().now().nanoseconds

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
