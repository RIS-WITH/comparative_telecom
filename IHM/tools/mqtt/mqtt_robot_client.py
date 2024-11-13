import paho.mqtt.client as mqtt
import struct
import time
import socket
import threading
import json
import argparse

# Load settings from JSON file
with open('settings.json', 'r') as f:
    settings = json.load(f)

# Extract MQTT settings from the JSON config
mqtt_settings = settings['mqttSettings']
MQTT_BROKER = mqtt_settings['url']
MQTT_PORT = 9001  # WebSocket port
MQTT_USERNAME = mqtt_settings['options']['username']
MQTT_PASSWORD = mqtt_settings['options']['password']
MQTT_TOPIC_CMD_VEL = "cmd_vel/geometry_msgs/TwistStamped"
MQTT_TOPIC_COMPLETION = "temp_completion/std_msgs/Int64"
MQTT_TOPIC_TIMESTAMP = "cmd_vel_time_stamp/interfaces/CommandTimestamp"

# Parse command-line arguments for robot IP and port
parser = argparse.ArgumentParser(description='MQTT Robot Client')
parser.add_argument('--robot_ip', type=str, required=True, help='IP address of the robot')
parser.add_argument('--robot_port', type=int, default=12345, help='Port of the robot')
args = parser.parse_args()

ROBOT_IP = args.robot_ip
ROBOT_PORT = args.robot_port

class YunoboMQTTClient:
    def __init__(self):
        # MQTT setup
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, transport="websockets")
        self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT)

        # Robot socket setup
        self.client_socket = self.setup_socket()

        # Start a background thread for socket handling
        self.stop_thread = False
        self.socket_thread = threading.Thread(target=self.socket_handler)
        self.socket_thread.start()

        # Start MQTT loop
        self.mqtt_client.loop_start()

    def setup_socket(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.setblocking(False)
        client_socket.connect_ex((ROBOT_IP, ROBOT_PORT))
        return client_socket

    def on_connect(self, client, userdata, flags, rc, properties=None):
        print("Connected to MQTT Broker with result code " + str(rc))
        client.subscribe(MQTT_TOPIC_CMD_VEL)

    def on_message(self, client, userdata, msg):
        T2 = int(time.time() * 1e9)
        received_message = msg.payload.decode('utf-8')
        if msg.topic == MQTT_TOPIC_CMD_VEL:
            # Handle the cmd_vel message (e.g., for controlling robot movement)
            self.handle_cmd_vel(received_message, T2)

    def handle_cmd_vel(self, received_message, T2):
        # Parse the received message and send command to the robot
        message = json.loads(received_message)
        linear_x = message['twist']['linear']['x']
        angular_z = message['twist']['angular']['z']
        command_id = self.extract_ns_from_header(message['header']['stamp'])
        self.send_cmd_vel_to_robot(linear_x, angular_z, command_id)
        self.publish_timestamp(command_id, 2, T2)

    def send_cmd_vel_to_robot(self, linear_x, angular_z, command_id):
        packed_data = struct.pack(">qff", command_id, linear_x, angular_z)
        try:
            self.client_socket.sendall(packed_data)
        except BlockingIOError:
            print("Warning: Failed to send command to robot.")

    def extract_ns_from_header(self, header_stamp):
        return int(header_stamp['sec'] * 1e9 + header_stamp['nanosec'])

    def socket_handler(self):
        while not self.stop_thread:
            try:
                data = self.client_socket.recv(16)
                if data:
                    self.process_received_data(data)
            except BlockingIOError:
                pass
            time.sleep(0.01)  # Polling delay

    def process_received_data(self, data):
        T4 = int(time.time() * 1e9)
        command_id, T3 = struct.unpack(">qq", data)

        # Publish completion and timestamp data
        self.mqtt_client.publish(MQTT_TOPIC_COMPLETION, str(command_id))
        self.publish_timestamp(command_id, 3, T3)
        self.publish_timestamp(command_id, 4, T4)

    def publish_timestamp(self, command_id, timestamp_index, timestamp_value):
        timestamp_message = {
            "command_id": command_id,
            "timestamp_index": timestamp_index,
            "timestamp": timestamp_value
        }
        self.mqtt_client.publish(MQTT_TOPIC_TIMESTAMP, json.dumps(timestamp_message))

    def stop(self):
        self.stop_thread = True
        self.socket_thread.join()
        self.client_socket.close()
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()

if __name__ == "__main__":
    client = YunoboMQTTClient()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Shutting down...")
        client.stop()
