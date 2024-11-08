#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/int64.hpp>
#include <interfaces/msg/command_timestamp.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>

class YunoboNode : public rclcpp::Node {
public:
    YunoboNode()
        : Node("yunobo_node"), stop_thread_(false) {
        
        // Declare the parameter for robot's IP address with a default value
        this->declare_parameter<std::string>("robot_ip", "127.0.0.1");
        this->get_parameter("robot_ip", robot_ip_);

        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

        // Setup socket
        setup_socket();

        // Setup publishers
        time_stamp_publisher_ = this->create_publisher<interfaces::msg::CommandTimestamp>(
            "/cmd_vel_time_stamp", qos_profile);
        temp_completion_publisher_ = this->create_publisher<std_msgs::msg::Int64>(
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

        // Make the socket non-blocking
        int flags = fcntl(client_socket_, F_GETFL, 0);
        fcntl(client_socket_, F_SETFL, flags | O_NONBLOCK);

        struct sockaddr_in server_address;
        std::memset(&server_address, 0, sizeof(server_address));
        server_address.sin_family = AF_INET;
        server_address.sin_port = htons(12345);

        if (inet_pton(AF_INET, robot_ip_.c_str(), &server_address.sin_addr) <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Invalid robot IP address.");
            return;
        }

        if (connect(client_socket_, (struct sockaddr*)&server_address, sizeof(server_address)) < 0 && errno != EINPROGRESS) {
            RCLCPP_ERROR(this->get_logger(), "Connection to robot failed.");
            return;
        }
    }

    void listener_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        int64_t T2 = get_current_time_in_ns();
        int64_t command_id = extract_ns_from_header(msg->header.stamp);

        send_cmd_vel_to_robot(msg, command_id);
        publish_timestamp(command_id, 2, T2);
    }

    void send_cmd_vel_to_robot(const geometry_msgs::msg::TwistStamped::SharedPtr msg, int64_t command_id) {
        int64_t network_command_id = htonll(command_id);
        float linear_x = msg->twist.linear.x;
        float angular_z = msg->twist.angular.z;

        char buffer[16];
        std::memcpy(buffer, &network_command_id, sizeof(network_command_id));
        std::memcpy(buffer + 8, &linear_x, sizeof(linear_x));
        std::memcpy(buffer + 12, &angular_z, sizeof(angular_z));

        if (send(client_socket_, buffer, sizeof(buffer), 0) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to send command to robot.");
        }
    }

    void socket_handler() {
        fd_set readfds;
        struct timeval tv;
        char data[16];

        while (!stop_thread_) {
            FD_ZERO(&readfds);
            FD_SET(client_socket_, &readfds);
            tv.tv_sec = 0;
            tv.tv_usec = 10000; // 10 ms timeout

            int activity = select(client_socket_ + 1, &readfds, NULL, NULL, &tv);

            if (activity > 0 && FD_ISSET(client_socket_, &readfds)) {
                int bytes_received = recv(client_socket_, data, sizeof(data), 0);
                if (bytes_received > 0) {
                    process_received_data(data);
                } else if (bytes_received < 0 && errno != EWOULDBLOCK) {
                    RCLCPP_ERROR(this->get_logger(), "Socket receive error.");
                }
            }
        }
    }

    void process_received_data(const char* data) {
        int64_t T4 = get_current_time_in_ns();
        int64_t command_id, T3;

        std::memcpy(&command_id, data, sizeof(command_id));
        command_id = ntohll(command_id);

        std::memcpy(&T3, data + 8, sizeof(T3));
        T3 = ntohll(T3);

        std_msgs::msg::Int64 msg;
        msg.data = command_id;
        temp_completion_publisher_->publish(msg);

        publish_timestamp(command_id, 3, T3);
        publish_timestamp(command_id, 4, T4);
    }

    void publish_timestamp(int64_t command_id, int timestamp_index, int64_t timestamp_value) {
        auto msg = interfaces::msg::CommandTimestamp();
        msg.command_id = command_id;
        msg.timestamp_index = timestamp_index;
        msg.timestamp = timestamp_value;
        time_stamp_publisher_->publish(msg);
    }

    int64_t get_current_time_in_ns() {
        return this->now().nanoseconds();
    }

    int64_t extract_ns_from_header(const rclcpp::Time& stamp) {
        return stamp.nanoseconds();
    }

    int client_socket_;
    std::string robot_ip_;
    rclcpp::Publisher<interfaces::msg::CommandTimestamp>::SharedPtr time_stamp_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr temp_completion_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_subscription_;

    std::thread socket_thread_;
    std::atomic<bool> stop_thread_;

    int64_t htonll(int64_t value) {
        return ((int64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
    }

    int64_t ntohll(int64_t value) {
        return ((int64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YunoboNode>());
    rclcpp::shutdown();
    return 0;
}
