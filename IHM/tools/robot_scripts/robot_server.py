import socket
import struct
import threading
import time
import select

class RobotNode:
    def __init__(self):
        self.stop_thread = False
        self.server_socket = None
        self.client_socket = None

        # Setup socket and start listening for commands in a separate thread
        self.setup_socket()
        self.listen_thread = threading.Thread(target=self.listen_for_commands)
        self.listen_thread.start()

    def setup_socket(self):
        try:
            # Create and bind the server socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Reuse the address
            self.server_socket.bind(('', 12345))  # Listen on port 12345, all network interfaces
            self.server_socket.listen(1)          # Listen for one connection
            self.server_socket.setblocking(False)  # Make server socket non-blocking
            print("RobotNode: Waiting for connections...")

            # Wait for a connection in a non-blocking way
            while not self.stop_thread:
                ready_to_read, _, _ = select.select([self.server_socket], [], [], 1)
                if ready_to_read:
                    self.client_socket, _ = self.server_socket.accept()
                    self.client_socket.setblocking(False)  # Make client socket non-blocking
                    print("RobotNode: Connected to YunoboNode")
                    break
        except socket.error as e:
            print("Socket setup error:", e)

    def listen_for_commands(self):
        while not self.stop_thread:
            try:
                if self.client_socket:
                    # Use select to check for readability
                    ready_to_read, _, _ = select.select([self.client_socket], [], [], 0.1)
                    if ready_to_read:
                        # Expecting 16 bytes (8 for command_id, 8 for velocities)
                        data = self.client_socket.recv(16)
                        if len(data) < 16:
                            print("Connection closed or error in receiving data.")
                            break  # Exit if connection closed or error occurs
                        self.process_command(data)
            except socket.error as e:
                print("Error receiving data:", e)
                break

    def process_command(self, data):
        # Unpack the received command
        command_id, linear_x, angular_z = struct.unpack('!qff', data)

        # Get current time for T3
        T3 = self.get_current_time_in_ns()

        # Pack command_id and T3 into completion data
        completion_data = struct.pack('!qq', command_id, T3)

        # Send the packed completion data back
        try:
            self.client_socket.sendall(completion_data)
        except socket.error as e:
            print("Error sending completion data:", e)

    def get_current_time_in_ns(self):
        # Returns the current time in nanoseconds
        return int(time.time() * 1e9)

    def close(self):
        # Close sockets and stop thread
        self.stop_thread = True
        if self.listen_thread.is_alive():
            self.listen_thread.join()
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()

if __name__ == '__main__':
    try:
        node = RobotNode()
        while True:
            time.sleep(1)  # Keep main thread alive
    except KeyboardInterrupt:
        print("Shutting down RobotNode.")
    finally:
        node.close()

