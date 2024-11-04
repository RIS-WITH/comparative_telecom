#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstring>
#include <unistd.h>

class RobotNode : public rclcpp::Node {
public:
    RobotNode()
        : Node("robot_node"), stop_thread_(false) {
        // Setup socket
        setup_socket();

        // Start listening for commands in a separate thread
        listen_thread_ = std::thread(&RobotNode::listen_for_commands, this);
    }

    ~RobotNode() {
        stop_thread_ = true;
        if (listen_thread_.joinable()) {
            listen_thread_.join();
        }
        close(server_socket_);
    }

private:
    void setup_socket() {
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket.");
            return;
        }

        struct sockaddr_in server_address;
        std::memset(&server_address, 0, sizeof(server_address));
        server_address.sin_family = AF_INET;
        server_address.sin_addr.s_addr = INADDR_ANY; // Bind to all interfaces
        server_address.sin_port = htons(12345);  // Listen on port 12345

        if (bind(server_socket_, (struct sockaddr*)&server_address, sizeof(server_address)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket.");
            return;
        }

        listen(server_socket_, 1);  // Listen for one connection
        RCLCPP_INFO(this->get_logger(), "RobotNode: Waiting for connections...");

        client_socket_ = accept(server_socket_, nullptr, nullptr); // Accept a connection
        if (client_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to accept connection.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "RobotNode: Connected to YunoboNode");
    }

    void listen_for_commands() {
        while (!stop_thread_) {
            char data[16];  // Expecting 16 bytes (8 for command_id, 8 for velocities)
            int bytes_received = recv(client_socket_, data, sizeof(data), 0);
            if (bytes_received <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Connection closed or error in receiving data.");
                break;  // Exit if the connection is closed or an error occurs
            }
            process_command(data);
        }
    }

    void process_command(const char* data) {
        uint64_t command_id;
        float linear_x, angular_z;

        // Unpack the received command
        std::memcpy(&command_id, data, sizeof(command_id));
        command_id = ntohll(command_id);  // Convert from network byte order
        std::memcpy(&linear_x, data + 8, sizeof(linear_x));
        std::memcpy(&angular_z, data + 12, sizeof(angular_z));

        // Log the received command
        //RCLCPP_INFO(this->get_logger(), "RobotNode: Executing command %lu with velocities: linear_x=%f, angular_z=%f",
                    //command_id, linear_x, angular_z);

        // Get current time for T3 and convert it to network byte order
        uint64_t T3 = htonll(get_current_time_in_ns());

        // Pack command_id and T3 into completion_data
        char completion_buffer[16];
        uint64_t network_command_id = htonll(command_id);  // Convert command_id to network byte order
        std::memcpy(completion_buffer, &network_command_id, sizeof(network_command_id));
        std::memcpy(completion_buffer + 8, &T3, sizeof(T3));

        // Send the packed completion data back
        send(client_socket_, completion_buffer, sizeof(completion_buffer), 0);
    }


    int64_t get_current_time_in_ns() {
        return this->now().nanoseconds();
    }

    // Helper functions for network byte order
    uint64_t htonll(uint64_t value) {
        return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
    }

    uint64_t ntohll(uint64_t value) {
        return ((uint64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
    }

    // Member variables
    int server_socket_;
    int client_socket_;
    std::thread listen_thread_;
    std::atomic<bool> stop_thread_;  // Thread-safe flag for stopping the thread
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNode>());
    rclcpp::shutdown();
    return 0;
}
