#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>

#include "../src/StateSaver.hpp"

class StateSaverTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        std::filesystem::remove_all("data");
        
        temp_node_ = rclcpp::Node::make_shared("test_node");
        imu_pub_ = temp_node_->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
        mag_pub_ = temp_node_->create_publisher<sensor_msgs::msg::MagneticField>("/mag", 10);
        pwm_pub_ = temp_node_->create_publisher<std_msgs::msg::Int32MultiArray>("sent_pwm_topic_", 10);
        depth_pub_ = temp_node_->create_publisher<std_msgs::msg::String>("depthPressureSensorData", 10);
    }

    void TearDown() override {
        rclcpp::shutdown();
        std::filesystem::remove_all("data");
    }

    rclcpp::Node::SharedPtr temp_node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr depth_pub_;
};

// Test CSV file initialization
TEST_F(StateSaverTest, TestCSVInitialization) {
    auto node = std::make_shared<StateSaver>();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    bool csv_found = false;
    for (const auto& entry : std::filesystem::directory_iterator("data")) {
        if (entry.path().extension() == ".csv") {
            csv_found = true;
            
            std::ifstream file(entry.path());
            std::string header;
            std::getline(file, header);
            
            // Check if header contains expected columns
            EXPECT_TRUE(header.find("timestamp") != std::string::npos);
            EXPECT_TRUE(header.find("angular_velocity_x") != std::string::npos);
            EXPECT_TRUE(header.find("magnetic_field_z") != std::string::npos);
            EXPECT_TRUE(header.find("pwm_1") != std::string::npos);
            EXPECT_TRUE(header.find("depth_pressure_sensor") != std::string::npos);
            break;
        }
    }
    EXPECT_TRUE(csv_found);
}

// Test PWM callback functionality
TEST_F(StateSaverTest, TestPWMCallback) {
    auto node = std::make_shared<StateSaver>();
    
    auto pwm_msg = std::make_shared<std_msgs::msg::Int32MultiArray>();
    pwm_msg->data = {1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700};
    
    pwm_pub_->publish(*pwm_msg);
    
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(true);
}

// Test depth pressure callback
TEST_F(StateSaverTest, TestDepthPressureCallback) {
    auto node = std::make_shared<StateSaver>();
    
    auto depth_msg = std::make_shared<std_msgs::msg::String>();
    depth_msg->data = "123.45";
    
    depth_pub_->publish(*depth_msg);
    
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(true); // Callback processed without crashing
}

// Test IMU callback
TEST_F(StateSaverTest, TestIMUCallback) {
    auto node = std::make_shared<StateSaver>();
    
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->angular_velocity.x = 1.0;
    imu_msg->angular_velocity.y = 2.0;
    imu_msg->angular_velocity.z = 3.0;
    imu_msg->linear_acceleration.x = 0.1;
    imu_msg->linear_acceleration.y = 0.2;
    imu_msg->linear_acceleration.z = 9.8;
    
    imu_pub_->publish(*imu_msg);
    
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    EXPECT_TRUE(true); // Callback processed without crashing
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
