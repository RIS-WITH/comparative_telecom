import rclpy
from rclpy.node import Node
import socket
import struct

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # Setup socket for client connection to YunoboNode
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect(('yunobo', 12345))  # Connect to YunoboNode's IP and port
        self.get_logger().info("RobotNode: Connected to YunoboNode")

        # Dictionary to store T3 values per command_id
        self.command_timestamps = {}

        # Start receiving data in a separate thread
        self.receive_data()

    def receive_data(self):
        while True:
            try:
                # First receive command ID (8 bytes)
                data = self.receive_full_data(8)  # Expecting 8 bytes (1 * 8 bytes)
                command_id = struct.unpack('!Q', data)[0]  # Unpack command ID

                # Receive velocities (8 bytes)
                velocity_data = self.receive_full_data(8)  # Expecting 8 bytes (2 * 4 bytes)
                linear_x, angular_z = struct.unpack('!ff', velocity_data)  # Unpack velocities

                # Now calculate T3, the time when the message has been fully received
                T3 = self.get_current_time_in_ns()  # Get current time for T3

                # Store T3 in the dictionary associated with the command_id
                self.command_timestamps[command_id] = T3

                self.get_logger().info(f"RobotNode: Received command ID: {command_id}, Velocities: ({linear_x}, {angular_z}), T3: {T3}")

                # Simulate command execution
                self.simulate_command_execution(command_id)

                # Send completion message back to YunoboNode
                self.send_completion(command_id)
            except Exception as e:
                self.get_logger().error(f"RobotNode: Error receiving data: {e}")
                break

    def receive_full_data(self, expected_bytes):
        """Receive data from the socket until the expected number of bytes is received."""
        data = bytearray()
        while len(data) < expected_bytes:
            packet = self.client_socket.recv(expected_bytes - len(data))
            if not packet:
                raise ConnectionError("Connection closed by remote host")
            data.extend(packet)
        return bytes(data)

    def send_completion(self, command_id):
        # Retrieve the correct T3 value for the given command_id
        if command_id in self.command_timestamps:
            T3 = self.command_timestamps.pop(command_id)  # Remove T3 from the dictionary once used
            completion_data = struct.pack('!QQ', command_id, T3)
            self.get_logger().info(f"RobotNode: Sending completion for Command ID: {command_id} with T3: {T3}")
            self.client_socket.sendall(completion_data)
        else:
            self.get_logger().error(f"RobotNode: T3 not found for Command ID: {command_id}")

    def simulate_command_execution(self, command_id):
        # Simulate a delay or processing for the command execution if necessary
        pass

    def get_current_time_in_ns(self):
        return self.get_clock().now().nanoseconds

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
