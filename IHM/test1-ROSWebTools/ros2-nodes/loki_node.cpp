#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include "interfaces/msg/command_timestamp.hpp" // Replace with your actual package name

using namespace std::chrono_literals;

class LokiNode : public rclcpp::Node {
public:
    LokiNode()
        : Node("loki_node") {
        
        // Define QoS with best-effort reliability and keep last 10 messages in the queue
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();

        // Subscriptions with optimized QoS
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel",
            qos_profile,
            std::bind(&LokiNode::listener_callback, this, std::placeholders::_1)
        );

        completion_subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
            "/temp_completion",
            qos_profile,
            std::bind(&LokiNode::completion_callback, this, std::placeholders::_1)
        );

        // Publisher for custom message
        time_stamp_publisher_ = this->create_publisher<interfaces::msg::CommandTimestamp>(
            "/cmd_vel_time_stamp",
            qos_profile
        );
    }

private:
    void listener_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // Get current time in nanoseconds
        int64_t T1 = get_current_time_in_ns();

        // Extract command_id from header's stamp (time in nanoseconds)
        uint64_t command_id = extract_ns_from_header(msg->header.stamp);

        // Publish timestamp for T1 using the custom message
        publish_timestamp(command_id, 1, T1);
    }

    void completion_callback(const std_msgs::msg::UInt64::SharedPtr msg) {
        // Get current time in nanoseconds for T5
        int64_t T5 = get_current_time_in_ns();
        publish_timestamp(msg->data, 5, T5);
    }

    void publish_timestamp(uint64_t command_id, int timestamp_index, int64_t timestamp_value) {
        auto msg = interfaces::msg::CommandTimestamp(); // Create an instance of your custom message
        msg.command_id = command_id;
        msg.timestamp_index = timestamp_index;
        msg.timestamp = timestamp_value;
        time_stamp_publisher_->publish(msg);
    }

    int64_t get_current_time_in_ns() {
        return this->now().nanoseconds();
    }

    uint64_t extract_ns_from_header(const rclcpp::Time& stamp) {
        return stamp.nanoseconds();
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr completion_subscription_;
    rclcpp::Publisher<interfaces::msg::CommandTimestamp>::SharedPtr time_stamp_publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LokiNode>());
    rclcpp::shutdown();
    return 0;
}
