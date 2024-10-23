import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt64MultiArray, UInt64
from rclpy.time import Time

class LokiNode(Node):
    def __init__(self):
        super().__init__('loki_node')

        # Subscriptions
        self.create_subscription(TwistStamped, '/cmd_vel', self.listener_callback, 1)
        self.create_subscription(UInt64, '/temp_completion', self.completion_callback, 1)

        # Publisher
        self.time_stamp_publisher = self.create_publisher(UInt64MultiArray, '/cmd_vel_time_stamp', 10)

    def listener_callback(self, msg):
        # Get current time and convert to milliseconds
        T1 = self.get_current_time_in_ns()

        # Extract T0 from header's stamp (time in milliseconds)
        command_id = self.extract_ns_from_header(msg.header.stamp)

        self.get_logger().info(f'Loki received command with ID: {command_id}, T0: {command_id}')

        # Publish timestamp for T1
        self.publish_timestamp(command_id, 1, T1)
        self.get_logger().info(f'Loki received and processed command with ID: {command_id}, T1: {T1}')

    def completion_callback(self, msg):
        # Get current time in nanoseconds for T5
        T5 = self.get_current_time_in_ns()
        self.publish_timestamp(msg.data, 5, T5)
        self.get_logger().info(f'Loki received completion notice for command ID: {msg.data}, T5: {T5}')

    def publish_timestamp(self, command_id, timestamp_index, timestamp_value):
        msg = UInt64MultiArray(data=[command_id, timestamp_index, timestamp_value])
        self.time_stamp_publisher.publish(msg)
        self.get_logger().info(f'Loki published timestamp: {msg.data}')

    def get_current_time_in_ns(self):
        return self.get_clock().now().nanoseconds
    
    def extract_ns_from_header(self, stamp):
        return Time.from_msg(stamp).nanoseconds

def main(args=None):
    rclpy.init(args=args)
    node = LokiNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
