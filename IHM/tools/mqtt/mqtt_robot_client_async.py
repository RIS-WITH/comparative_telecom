import asyncio
import aiomqtt
import struct
import time
import json
import argparse
import msgpack

# Load settings from JSON file
with open('settings.json', 'r') as f:
    settings = json.load(f)

# Extract MQTT settings from the JSON config
mqtt_settings = settings['mqttSettings']
MQTT_BROKER = mqtt_settings['url']
MQTT_PORT = mqtt_settings['port']
MQTT_USERNAME = mqtt_settings['options']['username']
MQTT_PASSWORD = mqtt_settings['options']['password']
MQTT_TOPIC_CMD_VEL = "cmd_vel/geometry_msgs/TwistStamped"
MQTT_TOPIC_COMPLETION = "temp_completion/std_msgs/Int64"
MQTT_TOPIC_TIMESTAMP = "cmd_vel_time_stamp/interfaces/CommandTimestamp"
MQTT_TOPIC_CHATTER = "chatter/std_msgs/String"

# Parse command-line arguments for robot IP and port
parser = argparse.ArgumentParser(description='MQTT Robot Client')
parser.add_argument('--robot_ip', type=str, required=True, help='IP address of the robot')
parser.add_argument('--robot_port', type=int, default=12345, help='Port of the robot')
args = parser.parse_args()

ROBOT_IP = args.robot_ip
ROBOT_PORT = args.robot_port


class YunoboMQTTClient:
    def __init__(self):
        self.stop_event = asyncio.Event()
        self.reader = None
        self.writer = None
        self.client = None

    async def setup_socket(self):
        try:
            self.reader, self.writer = await asyncio.open_connection(ROBOT_IP, ROBOT_PORT)
        except Exception as e:
            raise

    async def start_mqtt_loop(self):
        try:
            async with aiomqtt.Client(
                hostname=MQTT_BROKER,
                port=MQTT_PORT,
                username=MQTT_USERNAME,
                password=MQTT_PASSWORD,
            ) as client:
                self.client = client
                await client.subscribe(MQTT_TOPIC_CMD_VEL)
                async for message in client.messages:
                    await self.on_message(message)
        except Exception as e:
            pass

    async def on_message(self, message):
        T2 = time.time_ns()
        try:
            received_message = msgpack.unpackb(message.payload)
            if str(message.topic) == MQTT_TOPIC_CMD_VEL:
                await self.handle_cmd_vel(received_message, T2)
        except Exception as e:
            pass

    async def handle_cmd_vel(self, received_message, T2):
        try:
            linear_x = received_message['twist']['linear']['x']
            angular_z = received_message['twist']['angular']['z']
            command_id = self.extract_ns_from_header(received_message['header']['stamp'])
            await self.send_cmd_vel_to_robot(linear_x, angular_z, command_id)
            await self.publish_timestamp(command_id, 2, T2)
        except Exception as e:
            pass

    async def send_cmd_vel_to_robot(self, linear_x, angular_z, command_id):
        try:
            packed_data = struct.pack(">qff", command_id, float(linear_x), float(angular_z))
            self.writer.write(packed_data)
            await self.writer.drain()
        except Exception as e:
            pass

    def extract_ns_from_header(self, header_stamp):
        return int(header_stamp['sec'] * 1e9 + header_stamp['nanosec'])

    async def socket_handler(self):
        try:
            while not self.stop_event.is_set():
                data = await self.reader.read(16)
                if data:
                    await self.process_received_data(data)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            pass

    async def process_received_data(self, data):
        try:
            T4 = time.time_ns()
            command_id, T3 = struct.unpack(">qq", data)
            await self.client.publish(MQTT_TOPIC_COMPLETION, msgpack.packb(command_id), qos=0)
            await self.publish_timestamp(command_id, 3, T3)
            await self.publish_timestamp(command_id, 4, T4)
        except Exception as e:
            pass

    async def publish_timestamp(self, command_id, timestamp_index, timestamp_value):
        try:
            timestamp_message = {
                "command_id": command_id,
                "timestamp_index": timestamp_index,
                "timestamp": timestamp_value
            }
            await self.client.publish(MQTT_TOPIC_TIMESTAMP, msgpack.packb(timestamp_message), qos=0)
        except Exception as e:
            pass

    async def stop(self):
        self.stop_event.set()
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()


async def main():
    client = YunoboMQTTClient()
    try:
        await client.setup_socket()
        await asyncio.gather(client.start_mqtt_loop(), client.socket_handler())
    except Exception as e:
        pass
    finally:
        await client.stop()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
