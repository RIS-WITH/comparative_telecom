import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import UInt64
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Import your custom message
from interfaces.msg import CommandTimestamp  # Replace 'your_package_name' with the actual package name

class LokiNode(Node):
    def __init__(self):
        super().__init__('loki_node')

        # Define QoS with best-effort reliability and keep last 10 messages in the queue
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions with optimized QoS
        self.create_subscription(TwistStamped, '/cmd_vel', self.listener_callback, qos_profile)
        self.create_subscription(UInt64, '/temp_completion', self.completion_callback, qos_profile)

        # Publisher for custom message
        self.time_stamp_publisher = self.create_publisher(CommandTimestamp, '/cmd_vel_time_stamp', qos_profile)

    def listener_callback(self, msg):
        # Get current time and convert to nanoseconds
        T1 = self.get_current_time_in_ns()

        # Extract command_id from header's stamp (time in nanoseconds)
        command_id = self.extract_ns_from_header(msg.header.stamp)

        # Publish timestamp for T1 using the custom message
        self.publish_timestamp(command_id, 1, T1)

    def completion_callback(self, msg):
        # Get current time in nanoseconds for T5
        T5 = self.get_current_time_in_ns()
        self.publish_timestamp(msg.data, 5, T5)

    def publish_timestamp(self, command_id, timestamp_index, timestamp_value):
        msg = CommandTimestamp()  # Create an instance of your custom message
        msg.command_id = command_id
        msg.timestamp_index = timestamp_index
        msg.timestamp = timestamp_value
        self.time_stamp_publisher.publish(msg)

    def get_current_time_in_ns(self):
        return self.get_clock().now().nanoseconds
    
    def extract_ns_from_header(self, stamp):
        return Time.from_msg(stamp).nanoseconds

def main(args=None):
    rclpy.init(args=args)
    node = LokiNode()

    # Use non-blocking spin to improve efficiency
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
