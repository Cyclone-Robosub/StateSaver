#include "SensorDataConfig.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <iomanip>

SensorsDataConfig::SensorsDataConfig(std::ofstream &outputStateFile)
    : Node("sensorsNode"), m_stateFile(outputStateFile) {
  callbackDepthPressure =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackIMU =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  callbackClTool =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  callbackManual =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  auto commandOptions = rclcpp::SubscriptionOptions();
  commandOptions.callback_group = callbackClTool;
  auto durationOptions = rclcpp::SubscriptionOptions();
  durationOptions.callback_group = callbackClTool;
  auto depthPressureOptions = rclcpp::SubscriptionOptions();
  depthPressureOptions.callback_group = callbackDepthPressure;
  auto imuOptions = rclcpp::SubscriptionOptions();
  imuOptions.callback_group = callbackIMU;
  std::cout << "Creating sensors subscriptions\n";
  auto magOptions = rclcpp::SubscriptionOptions();
  magOptions.callback_group = callbackIMU;
  // Fix: changed from rclcpp : SubscriptionOptions() to
  // rclcpp::SubscriptionOptions()
  auto odom_ins_ned_Options = rclcpp::SubscriptionOptions();
  odom_ins_ned_Options.callback_group = callbackIMU;
  auto ManualToggleOptions = rclcpp::SubscriptionOptions();
  ManualToggleOptions.callback_group = callbackManual;
  auto ManualOverrideOptions = rclcpp::SubscriptionOptions();
  ManualOverrideOptions.callback_group = callbackManual;

  depth_pressure_sensor_subscription_ =
      this->create_subscription<std_msgs::msg::String>(
          "depthPressureSensorData", rclcpp::QoS(5),
          std::bind(&SensorsDataConfig::depthPressureSensorCallback, this,
                    std::placeholders::_1),
          depthPressureOptions);

  // Priority
  // Need to input IMU initialization with ROS.

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", rclcpp::QoS(5),
      std::bind(&SensorsDataConfig::imuSensorCallback, this,
                std::placeholders::_1),
      imuOptions);
  mag_subscription_ =
      this->create_subscription<sensor_msgs::msg::MagneticField>(
          "mag", rclcpp::QoS(5),
          std::bind(&SensorsDataConfig::magCallback, this,
                    std::placeholders::_1),
          magOptions);

  // Please implement Kory's data as soon as possible
  odom_ins_ned_subscription =
      this->create_subscription<nav_msgs::msg::Odometry>(
          "roll_pitch_yaw", rclcpp::QoS(5),
          // Fix: Ensure callback function name matches
          std::bind(&SensorsDataConfig::Odom_insCallback, this,
                    std::placeholders::_1),
          odom_ins_ned_Options);

  position_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "position_topic", rclcpp::QoS(5),
      std::bind(&SensorsDataConfig::positionCallback, this,
                std::placeholders::_1),
      positionOptions);

  waypoint_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "waypoint_topic", rclcpp::QoS(5),
      std::bind(&SensorsDataConfig::waypointCallback, this,
                std::placeholders::_1),
      waypointOptions);

  

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SensorsDataConfig::writeDataToFile, this));
}

void SensorsDataConfig::depthPressureSensorCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  depthPressureData_ = msg->data;
}
void SensorsDataConfig::imuSensorCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  imuData_ = *msg;
}
void SensorsDataConfig::magCallback(
    const sensor_msgs::msg::MagneticField::SharedPtr msg) {
  magData_ = *msg;
}
void SensorsDataConfig::Odom_insCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  geometry_msgs::msg::Quaternion orientation_quat_msg =
      msg->pose.pose.orientation;
  tf2::Quaternion q;
  tf2::fromMsg(orientation_quat_msg, q);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_r, pitch_r, yaw_r);
}

void SensorsDataConfig::writeDataToFile() {
  auto now = std::chrono::system_clock::now();
  std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
  m_stateFile << std::put_time(std::localtime(&currentTime), "%F %T") << ",";
  m_stateFile << depthPressureData_ << ",";

  for (size_t i = 0; i < positionData_.data.size(); ++i) {
    m_stateFile << positionData_.data[i]
                << (i == positionData_.data.size() - 1 ? "" : ",");
  }
  m_stateFile << ",";
  for (size_t i = 0; i < waypointData_.data.size(); ++i) {
    m_stateFile << waypointData_.data[i]
                << (i == waypointData_.data.size() - 1 ? "" : ",");
  }
  

#  m_stateFile << imuData_.orientation.x << "," << imuData_.orientation.y << ","
#              << imuData_.orientation.z << "," << imuData_.orientation.w << ",";
#
#  // ADDED: Gyro (Angular Velocity)
#  m_stateFile << imuData_.angular_velocity.x << ","
#              << imuData_.angular_velocity.y << ","
#              << imuData_.angular_velocity.z << ",";
#
#  // ADDED: Accel (Linear Acceleration)
#  m_stateFile << imuData_.linear_acceleration.x << ","
#              << imuData_.linear_acceleration.y << ","
#              << imuData_.linear_acceleration.z << ",";
#
#  // ADDED: Mag (Magnetic Field)
#  m_stateFile << magData_.magnetic_field.x << "," << magData_.magnetic_field.y
#              << "," << magData_.magnetic_field.z << ",";
#
#  // ADDED: Roll, Pitch, Yaw
#  m_stateFile << roll_r << "," << pitch_r << "," << yaw_r << ",";
#
  for (size_t i = 0; i < pwmData_.data.size(); ++i) {
    m_stateFile << pwmData_.data[i]
                << (i == pwmData_.data.size() - 1 ? "" : ",");
  }
  m_stateFile << "\n";
  if (m_stateFile.tellp() > 100) {
    m_stateFile.flush();
    m_stateFile.clear();
    m_stateFile.seekp(0);
  }
}