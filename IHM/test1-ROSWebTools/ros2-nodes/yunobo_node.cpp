#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <interfaces/msg/command_timestamp.hpp> // Replace with actual package name
#include <chrono>
#include <thread>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>

class YunoboNode : public rclcpp::Node {
public:
    YunoboNode()
        : Node("yunobo_node"), stop_thread_(false) {
        
        // Declare the parameter for robot's IP address with a default value
        this->declare_parameter<std::string>("robot_ip", "127.0.0.1"); // Default to localhost for safety
        this->get_parameter("robot_ip", robot_ip_);

        // Define QoS with best-effort reliability
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

        // Setup socket
        setup_socket();

        // Setup publishers
        time_stamp_publisher_ = this->create_publisher<interfaces::msg::CommandTimestamp>(
            "/cmd_vel_time_stamp", qos_profile);
        temp_completion_publisher_ = this->create_publisher<std_msgs::msg::UInt64>(
            "/temp_completion", 1);

        // Subscribe to /cmd_vel topic
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel",
            qos_profile,
            std::bind(&YunoboNode::listener_callback, this, std::placeholders::_1)
        );

        // Start socket handling thread
        socket_thread_ = std::thread(&YunoboNode::socket_handler, this);
    }

    ~YunoboNode() {
        stop_thread_ = true;
        if (socket_thread_.joinable()) {
            socket_thread_.join();
        }
        close(client_socket_);
    }

private:
    void setup_socket() {
        client_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (client_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket.");
            return;
        }

        struct sockaddr_in server_address;
        std::memset(&server_address, 0, sizeof(server_address));
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(12345);  // Replace with actual port

        // Use the parameterized robot IP
        if (inet_pton(AF_INET, robot_ip_.c_str(), &server_address.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address.");
            return;
        }

        if (connect(client_socket_, (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Connection to robot failed.");
            return;
        }
    }

    void listener_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        // Get current time for T2
        int64_t T2 = get_current_time_in_ns();
        uint64_t command_id = extract_ns_from_header(msg->header.stamp);

        // Send command to robot
        send_cmd_vel_to_robot(msg, command_id);

        // Publish timestamp
        publish_timestamp(command_id, 2, T2);
    }

    void send_cmd_vel_to_robot(const geometry_msgs::msg::TwistStamped::SharedPtr msg, uint64_t command_id) {
        // Pack command_id, linear and angular velocities
        uint64_t network_command_id = htonll(command_id);  // Network byte order
        float linear_x = msg->twist.linear.x;
        float angular_z = msg->twist.angular.z;

        char buffer[16];
        std::memcpy(buffer, &network_command_id, sizeof(network_command_id));
        std::memcpy(buffer + 8, &linear_x, sizeof(linear_x));
        std::memcpy(buffer + 12, &angular_z, sizeof(angular_z));

        send(client_socket_, buffer, sizeof(buffer), 0);
    }

    void socket_handler() {
        while (!stop_thread_) {
            char data[16];  // Expecting 16 bytes (2 * 8 bytes)
            int bytes_received = recv(client_socket_, data, sizeof(data), 0);
            if (bytes_received > 0) {
                process_received_data(data);
            } else if (bytes_received < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to receive completion message.");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // Avoid busy waiting
        }
    }

    void process_received_data(const char* data) {
        int64_t T4 = get_current_time_in_ns();
        uint64_t command_id, T3;

        std::memcpy(&command_id, data, sizeof(command_id));
        command_id = ntohll(command_id);  // Convert from network byte order

        std::memcpy(&T3, data + 8, sizeof(T3));
        T3 = ntohll(T3);  // Convert from network byte order

        // Publish the command completion message
        std_msgs::msg::UInt64 msg;
        msg.data = command_id;
        temp_completion_publisher_->publish(msg);

        publish_timestamp(command_id, 3, T3);
        publish_timestamp(command_id, 4, T4);
    }

    void publish_timestamp(uint64_t command_id, int timestamp_index, int64_t timestamp_value) {
        auto msg = interfaces::msg::CommandTimestamp();
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

    // Member variables
    int client_socket_;
    std::string robot_ip_;  // Variable to hold the robot IP address
    rclcpp::Publisher<interfaces::msg::CommandTimestamp>::SharedPtr time_stamp_publisher_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr temp_completion_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;

    std::thread socket_thread_;
    std::atomic<bool> stop_thread_;  // Thread-safe flag for stopping the thread

    // Helper functions for network byte order
    uint64_t htonll(uint64_t value) {
        return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
    }

    uint64_t ntohll(uint64_t value) {
        return ((uint64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YunoboNode>());
    rclcpp::shutdown();
    return 0;
}
