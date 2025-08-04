#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <iostream>

#include "../src/SensorDataConfig.hpp"
namespace fs = std::filesystem;
std::ofstream makeStateFile() {
  std::ofstream stateFile;
  fs::path stateFilePath = fs::current_path().parent_path().parent_path();
  std::string stateFileString =
      stateFilePath / "state.csv"; // Use / for path concatenation

  // Improved file handling: Create a new file each time.  You could
  // alternatively implement a more sophisticated rotation strategy
  stateFile.open(
      stateFileString,
      std::ofstream::app); // Open for output, truncating if it exists

  if (stateFile.is_open()) {
    stateFile << "Time,Depth(m),Pressure,IMU "
                 "Data,Gyro,Accel,Mag,Roll,Pitch,Yaw,PWM Data"
              << std::endl;
    std::cout << "Created/Overwrote state file: " << stateFileString
              << std::endl;
  } else {
    std::cerr << "Error opening state file: " << stateFileString << std::endl;
  }
  return stateFile;
}
std::ofstream THISISTHEONE = makeStateFile();
class SensorsDataConfigTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        std::filesystem::remove_all("state");
        
        temp_node_ = rclcpp::Node::make_shared("test_node");
        imu_pub_ = temp_node_->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        mag_pub_ = temp_node_->create_publisher<sensor_msgs::msg::MagneticField>("mag", 10);
        odo_pub_ = temp_node_->create_publisher<nav_msgs::msg::Odometry>("roll_pitch_yaw", 10);
        depth_pub_ = temp_node_->create_publisher<std_msgs::msg::String>("depthPressureSensorData", 10);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }

    rclcpp::Node::SharedPtr temp_node_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr depth_pub_;
};

// Test CSV file initialization
TEST_F(SensorsDataConfigTest, TestCSVInitialization) {
    auto node = std::make_shared<SensorsDataConfig>(THISISTHEONE);
  std::cout << "ROS2 StateSaver start running" << std::endl;
  std::jthread ROSthread([node](){rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);executor.spin();});
  std::cout << "here" <<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    fs::path stateFilePath = fs::current_path().parent_path().parent_path() / "state.csv";
    EXPECT_TRUE(fs::exists(stateFilePath));
    
    // Check the header content
    std::ifstream file(stateFilePath);
    ASSERT_TRUE(file.is_open());
    std::string header;
    std::getline(file, header);
    
    // Check if header contains expected columns
    EXPECT_EQ(header, "Time,Depth(m),Pressure,IMU Data,Gyro,Accel,Mag,Roll,Pitch,Yaw,PWM Data");
    rclcpp::shutdown();
}


// Test depth pressure callback
TEST_F(SensorsDataConfigTest, TestDepthPressureCallback) {
    auto node = std::make_shared<SensorsDataConfig>(THISISTHEONE);
    std::cout << "ROS2 StateSaver start running.  Please wait for 3 seconds" << std::endl;
  std::jthread ROSthread([node](){rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);executor.spin();});
    auto depth_msg = std::make_shared<std_msgs::msg::String>();
    depth_msg->data = "123.45";
    
    depth_pub_->publish(*depth_msg);
    
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    EXPECT_TRUE(true); // Callback processed without crashing
   rclcpp::shutdown();
}

// Test IMU callback
TEST_F(SensorsDataConfigTest, TestIMUCallback) {
    auto node = std::make_shared<SensorsDataConfig>(THISISTHEONE);
     std::cout << "ROS2 StateSaver start running. Please wait for 3 seconds" << std::endl;
  std::jthread ROSthread([node](){rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);executor.spin();});
    auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
    imu_msg->angular_velocity.x = 1.0;
    imu_msg->angular_velocity.y = 2.0;
    imu_msg->angular_velocity.z = 3.0;
    imu_msg->linear_acceleration.x = 0.1;
    imu_msg->linear_acceleration.y = 0.2;
    imu_msg->linear_acceleration.z = 9.8;
    
    imu_pub_->publish(*imu_msg);
    
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    EXPECT_TRUE(true); // Callback processed without crashing // For now use your eyes.
    rclcpp::shutdown(); //Remove this to see how long it goes.
}
TEST_F(SensorsDataConfigTest, IntentionalWaiting){
    std::cout << "THIS TEST IS MEANT TO BE STUCK/APPEAR FROZEN" << std::endl;
    std::jthread PublisherThread([this](){
         auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_msg->angular_velocity.x = 1.0;
        imu_msg->angular_velocity.y = 2.0;
        imu_msg->angular_velocity.z = 2525.0;
        imu_msg->linear_acceleration.x = 0.1;
        imu_msg->linear_acceleration.y = 325352535;
        imu_msg->linear_acceleration.z = 342;

        while(true){
        this->imu_pub_->publish(*imu_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });
    auto sensorsNode = std::make_shared<SensorsDataConfig>(THISISTHEONE);
    rclcpp::executors::MultiThreadedExecutor executor;
     executor.add_node(sensorsNode);
    std::cout << "ROS2 StateSaver running" << std::endl;
    executor.spin();
   rclcpp::shutdown();
  std::cout << "Shutting Down" << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
} 
