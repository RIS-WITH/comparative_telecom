#include <iostream>
#include <cstring>
#include <thread>
#include <atomic>
#include <arpa/inet.h>
#include <unistd.h>
#include <ctime>
#include <signal.h>

#define PORT 12345

int server_socket, client_socket;
std::thread listen_thread;
std::atomic<bool> stop_thread(false);

// Helper function to get the current time in nanoseconds
int64_t get_current_time_in_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1e9 + ts.tv_nsec;
}

// Network byte order conversion for 64-bit integers
uint64_t htonll(uint64_t value) {
    return ((uint64_t)htonl(value & 0xFFFFFFFF) << 32) | htonl(value >> 32);
}

uint64_t ntohll(uint64_t value) {
    return ((uint64_t)ntohl(value & 0xFFFFFFFF) << 32) | ntohl(value >> 32);
}

// Function to process the command and send the response
void process_command(const char *data) {
    uint64_t command_id;
    float linear_x, angular_z;

    // Unpack received data (8 bytes for command_id, 4 bytes each for linear_x and angular_z)
    std::memcpy(&command_id, data, sizeof(command_id));
    command_id = ntohll(command_id); // Convert to host byte order
    std::memcpy(&linear_x, data + 8, sizeof(linear_x));
    std::memcpy(&angular_z, data + 12, sizeof(angular_z));

    // Get current time for T3
    uint64_t T3 = htonll(get_current_time_in_ns());

    // Pack command_id and T3 into response data
    char completion_data[16];
    uint64_t network_command_id = htonll(command_id); // Convert command_id to network byte order
    std::memcpy(completion_data, &network_command_id, sizeof(network_command_id));
    std::memcpy(completion_data + 8, &T3, sizeof(T3));

    // Send the packed completion data back to the client
    if (send(client_socket, completion_data, sizeof(completion_data), 0) < 0) {
        std::cerr << "Error sending completion data" << std::endl;
    }
}

// Thread function to listen for incoming commands
void listen_for_commands() {
    while (!stop_thread) {
        char data[16]; // Expecting 16 bytes (8 for command_id, 8 for velocities)
        int bytes_received = recv(client_socket, data, sizeof(data), 0);
        if (bytes_received <= 0) {
            std::cerr << "Connection closed or error in receiving data" << std::endl;
            break;
        }
        process_command(data);
    }
}

// Setup socket to listen for incoming connections
void setup_socket() {
    struct sockaddr_in server_address;

    // Create server socket
    server_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (server_socket < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Bind the socket to the port
    std::memset(&server_address, 0, sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = INADDR_ANY; // Bind to all interfaces
    server_address.sin_port = htons(PORT);

    if (bind(server_socket, (struct sockaddr *)&server_address, sizeof(server_address)) < 0) {
        std::cerr << "Failed to bind socket" << std::endl;
        close(server_socket);
        exit(EXIT_FAILURE);
    }

    // Listen for incoming connections
    listen(server_socket, 1);
    std::cout << "RobotNode: Waiting for connections on port " << PORT << "..." << std::endl;

    // Accept a connection
    client_socket = accept(server_socket, nullptr, nullptr);
    if (client_socket < 0) {
        std::cerr << "Failed to accept connection" << std::endl;
        close(server_socket);
        exit(EXIT_FAILURE);
    }

    std::cout << "RobotNode: Connected to client" << std::endl;
}

// Signal handler for clean exit on Ctrl+C
void handle_sigint(int sig) {
    stop_thread = true;
    close(client_socket);
    close(server_socket);
    std::cout << "\nShutting down RobotNode." << std::endl;
    exit(0);
}

int main() {
    // Register signal handler for Ctrl+C
    signal(SIGINT, handle_sigint);

    // Setup socket and start listening for commands
    setup_socket();
    listen_thread = std::thread(listen_for_commands);

    // Wait for the listening thread to finish (or handle Ctrl+C signal to exit)
    listen_thread.join();

    // Clean up
    close(client_socket);
    close(server_socket);
    return 0;
}
