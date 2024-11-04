import rclpy
from rclpy.node import Node
import socket
import struct
import threading

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Setup socket for client connection to YunoboNode
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('yunobo', 12345))  # Connect to YunoboNode's IP and port

        # Start receiving data in a separate thread
        threading.Thread(target=self.receive_data, daemon=True).start()

    def receive_data(self):
        """Receive data from the socket continuously."""
        try:
            while True:
                # Receive command ID (8 bytes) and velocity (8 bytes) at once
                data = self.receive_full_data(16)
                command_id, linear_x, angular_z = struct.unpack('!Qff', data)

                # Capture T3 as the current time when data is fully received
                T3 = self.get_current_time_in_ns()

                # Send completion message back to YunoboNode
                self.send_completion(command_id, T3)

        except Exception as e:
            self.get_logger().error(f"RobotNode: Error receiving data: {e}")

    def receive_full_data(self, expected_bytes):
        """Receive data from the socket until the expected number of bytes is received."""
        data = bytearray()
        while len(data) < expected_bytes:
            packet = self.client_socket.recv(expected_bytes - len(data))
            if not packet:
                raise ConnectionError("Connection closed by remote host")
            data.extend(packet)
        return bytes(data)

    def send_completion(self, command_id, T3):
        """Send the completion message back to YunoboNode with the command ID and T3."""
        completion_data = struct.pack('!QQ', command_id, T3)
        self.client_socket.sendall(completion_data)

    def get_current_time_in_ns(self):
        """Return the current time in nanoseconds."""
        return self.get_clock().now().nanoseconds

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
