#! /usr/bin/env python3

import rclpy
from comparative_telecom.MQTT import ROSMQTTBridge

def main(args=None):
    rclpy.init(args=args)
    ros_mqtt_bridge = ROSMQTTBridge()
    rclpy.spin(ros_mqtt_bridge)

    # Nettoyage
    ros_mqtt_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
