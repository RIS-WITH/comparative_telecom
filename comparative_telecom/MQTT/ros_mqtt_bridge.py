import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import logging
import uuid

# Configurer la journalisation
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

class ROSMQTTBridge(Node):
    def __init__(self):
        super().__init__('ros_mqtt_bridge')

        # Initialisation du client MQTT
        self.mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION1,client_id="ros_mqtt_bridge")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.mqtt_client.on_publish = self.on_publish
        self.mqtt_client.on_message = self.on_message

        self.mqtt_client.connect("localhost", 1883, 60)
        self.mqtt_client.loop_start()

        # Souscription aux topics ROS
        self.chatter_subscription = self.create_subscription(
            String,
            'chatter',
            self.chatter_callback,
            10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
       

        # Publishers ROS
        self.ros_chatter_publisher = self.create_publisher(String, 'chatter', 10)
        self.ros_cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Souscription aux topics MQTT
        self.mqtt_client.subscribe("ros/chatter")
        self.mqtt_client.subscribe("ros/cmd_vel")

        # Générer un ID unique pour identifier les messages de ce noeud
        self.node_id = str(uuid.uuid4())

    def on_connect(self, client, userdata, flags, rc):
        logger.debug(f"Connected to MQTT broker with result code {rc}")

    def on_disconnect(self, client, userdata, rc):
        logger.debug(f"Disconnected from MQTT broker with result code {rc}")

    def on_publish(self, client, userdata, mid):
        # logger.debug(f"Message {mid} published to MQTT broker")
        pass

    def on_message(self, client, userdata, msg):
        # logger.debug(f"Received message from MQTT topic {msg.topic}")
        try:
            data = json.loads(msg.payload)
            if msg.topic == "ros/chatter":
                if data.get("origin") != self.node_id:
                    ros_msg = String()
                    ros_msg.data = data["chatter"]
                    self.ros_chatter_publisher.publish(ros_msg)
                    logger.debug("Published message to ROS topic /chatter")
                else:
                    logger.debug("Ignored message from self to prevent loop")
            elif msg.topic == "ros/cmd_vel":
                if data.get("origin") != self.node_id:
                    ros_msg = Twist()
                    ros_msg.linear.x = data["linear"]["x"]
                    ros_msg.linear.y = data["linear"]["y"]
                    ros_msg.linear.z = data["linear"]["z"]
                    ros_msg.angular.x = data["angular"]["x"]
                    ros_msg.angular.y = data["angular"]["y"]
                    ros_msg.angular.z = data["angular"]["z"]
                    self.ros_cmd_vel_publisher.publish(ros_msg)
                    logger.debug("Published message to ROS topic /cmd_vel")
                else:
                    logger.debug("Ignored message from self to prevent loop")
        except json.JSONDecodeError as e:
            logger.error(f"Failed to decode JSON message: {e}")

    def chatter_callback(self, msg):
        logger.debug(f"Received message from /chatter: {msg.data}")
        payload = {"chatter": msg.data, "origin": self.node_id}
        self.mqtt_client.publish("ros/chatter", json.dumps(payload))

    def scan_callback(self, msg):
        # logger.debug("Received message from /scan")
        scan_data = {
            "header": {
                "stamp": {
                    "sec": msg.header.stamp.sec,
                    "nanosec": msg.header.stamp.nanosec
                },
                "frame_id": msg.header.frame_id
            },
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "time_increment": msg.time_increment,
            "scan_time": msg.scan_time,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "ranges": list(msg.ranges),
            "intensities": list(msg.intensities)
        }
        self.mqtt_client.publish("ros/scan", json.dumps(scan_data))

   
