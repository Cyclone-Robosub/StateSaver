#ifndef SENSORDATACONFIG_HPP
#define SENSORDATACONFIG_HPP

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>

// Include for nav_msgs/msg/Odometry
#include <nav_msgs/msg/odometry.hpp>

// Includes for tf2 and geometry_msgs conversions
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // Required for tf2::fromMsg

// Standard ROS message types
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>

class SensorsDataConfig : public rclcpp::Node {
public:
  // Constructor
  SensorsDataConfig(std::ofstream &stateFile);

  // Public method to write data to file
  void writeDataToFile();

private:
  // Callback groups for managing execution
  rclcpp::CallbackGroup::SharedPtr callbackDepthPressure;
  rclcpp::CallbackGroup::SharedPtr callbackIMU;
  rclcpp::CallbackGroup::SharedPtr callbackClTool;
  rclcpp::CallbackGroup::SharedPtr callbackManual;

  // Subscriptions
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
      depth_pressure_sensor_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr
      mag_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      odom_ins_ned_subscription;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
      CLTool_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr duration_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Manual_Control_sub;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Manual_Override_sub;

  // Output file stream reference
  std::ofstream &m_stateFile;

  // Data storage variables for sensor messages
  std::string depthPressureData_;
  sensor_msgs::msg::Imu imuData_;
  sensor_msgs::msg::MagneticField magData_;
  std_msgs::msg::Int32MultiArray pwmData_;
  std_msgs::msg::Float32MultiArray positionData_;
  std_msgs::msg::Float32MultiArray waypointData_;

  // Variables to store Roll, Pitch, Yaw from Odometry callback
  double roll_r, pitch_r, yaw_r;

  // Other data variables
  int64_t durationData_;
  bool manualControlData_;
  bool manualOverrideTriggered_ = false; // Initialized to false

  // Timer for periodic data writing
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback functions for subscriptions
  void pwmCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void positionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void waypointCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg);
  void imuSensorCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
  void PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void durationCallback(const std_msgs::msg::Int64::SharedPtr msg);
  void ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void ManualOverrideCallback(const std_msgs::msg::Empty::SharedPtr msg);
  // Callback for nav_msgs::msg::Odometry
  void Odom_insCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // SENSORDATACONFIG_HPP