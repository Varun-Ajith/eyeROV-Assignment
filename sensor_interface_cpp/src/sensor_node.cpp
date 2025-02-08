#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int16.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <arpa/inet.h>
#include <unistd.h>

using namespace std::chrono_literals;

class SensorNode : public rclcpp::Node {
public:
    SensorNode() : Node("sensor_node") {
        // Declare parameters
        this->declare_parameter("interval", 1000);  // Default interval in milliseconds
        this->declare_parameter("sensor_ip", "127.0.0.1");  // Sensor IP
        this->declare_parameter("sensor_port", 2000);  // Sensor port

        // Create publishers
        supply_voltage_pub_ = this->create_publisher<std_msgs::msg::UInt16>("/sensor/supply_voltage", 10);
        env_temp_pub_ = this->create_publisher<std_msgs::msg::Int16>("/sensor/env_temp", 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Int16>("/sensor/yaw", 10);
        pitch_pub_ = this->create_publisher<std_msgs::msg::Int16>("/sensor/pitch", 10);
        roll_pub_ = this->create_publisher<std_msgs::msg::Int16>("/sensor/roll", 10);

        // Connect to the sensor
        connect_to_sensor();

        // Start the receive thread
        receive_thread_ = std::thread(&SensorNode::receive_data, this);
    }

    ~SensorNode() {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
        if (sensor_socket_ != -1) {
            close(sensor_socket_);
        }
    }

private:
    void connect_to_sensor() {
        // Get sensor IP and port from parameters
        std::string sensor_ip = this->get_parameter("sensor_ip").as_string();
        int sensor_port = this->get_parameter("sensor_port").as_int();

        // Create socket
        sensor_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sensor_socket_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        // Connect to the sensor
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(sensor_port);
        inet_pton(AF_INET, sensor_ip.c_str(), &server_addr.sin_addr);

        if (connect(sensor_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to sensor");
            close(sensor_socket_);
            sensor_socket_ = -1;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Connected to sensor at %s:%d", sensor_ip.c_str(), sensor_port);

        // Send start command
        int interval = this->get_parameter("interval").as_int();
        send_start_command(interval);
    }

    void send_start_command(int interval) {
        if (sensor_socket_ == -1) {
            return;
        }

        // Convert interval to little-endian bytes
        uint16_t interval_le = htons(static_cast<uint16_t>(interval));

        // Increase buffer size to 10 to accommodate the full command
        char command[10];
        snprintf(command, sizeof(command), "#03%04X\r\n", interval_le);

        // Send command
        if (send(sensor_socket_, command, strlen(command), 0) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send start command");
            close(sensor_socket_);
            sensor_socket_ = -1;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sent start command with interval %dms", interval);
    }

    void receive_data() {
        char buffer[1024];
        while (running_ && sensor_socket_ != -1) {
            // Receive data
            ssize_t bytes_received = recv(sensor_socket_, buffer, sizeof(buffer) - 1, 0);
            if (bytes_received <= 0) {
                RCLCPP_ERROR(this->get_logger(), "Connection closed by sensor");
                close(sensor_socket_);
                sensor_socket_ = -1;
                break;
            }

            buffer[bytes_received] = '\0';
            std::string data(buffer);

            // Process messages
            size_t pos;
            while ((pos = data.find("\r\n")) != std::string::npos) {
                std::string message = data.substr(0, pos);
                data.erase(0, pos + 2);

                if (message.size() >= 3 && message[0] == '$' && message[1] == '1' && message[2] == '1') {
                    process_status_message(message.substr(3));
                }
            }
        }
    }

    void process_status_message(const std::string& payload) {
        if (payload.size() != 20) {  // 10 bytes = 20 hex characters
            RCLCPP_WARN(this->get_logger(), "Invalid payload length: %zu", payload.size());
            return;
        }

        // Convert hex string to bytes
        uint8_t data[10];
        for (size_t i = 0; i < 10; ++i) {
            data[i] = static_cast<uint8_t>(std::stoul(payload.substr(i * 2, 2), nullptr, 16));
        }

        // Unpack values (little-endian format)
        uint16_t supply_voltage = (data[1] << 8) | data[0];
        int16_t env_temp = (data[3] << 8) | data[2];
        int16_t yaw = (data[5] << 8) | data[4];
        int16_t pitch = (data[7] << 8) | data[6];
        int16_t roll = (data[9] << 8) | data[8];

        // Publish values
        auto supply_voltage_msg = std_msgs::msg::UInt16();
        supply_voltage_msg.data = supply_voltage;
        supply_voltage_pub_->publish(supply_voltage_msg);

        auto env_temp_msg = std_msgs::msg::Int16();
        env_temp_msg.data = env_temp;
        env_temp_pub_->publish(env_temp_msg);

        auto yaw_msg = std_msgs::msg::Int16();
        yaw_msg.data = yaw;
        yaw_pub_->publish(yaw_msg);

        auto pitch_msg = std_msgs::msg::Int16();
        pitch_msg.data = pitch;
        pitch_pub_->publish(pitch_msg);

        auto roll_msg = std_msgs::msg::Int16();
        roll_msg.data = roll;
        roll_pub_->publish(roll_msg);

        // Log the values
        RCLCPP_INFO(this->get_logger(),
            "Published: voltage=%umV, temp=%.1f°C, yaw=%.1f°, pitch=%.1f°, roll=%.1f°",
            supply_voltage, env_temp / 10.0f, yaw / 10.0f, pitch / 10.0f, roll / 10.0f
        );
    }

    // Member variables
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr supply_voltage_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr env_temp_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr yaw_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pitch_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr roll_pub_;

    int sensor_socket_ = -1;
    std::thread receive_thread_;
    bool running_ = true;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}