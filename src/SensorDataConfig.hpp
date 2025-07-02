#ifndef SENSORDATACONFIG_HPP
#define SENSORDATACONFIG_HPP
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/string.hpp>

class SensorsDataConfig : public rclcpp::Node {
public:
  SensorsDataConfig(std::ofstream &stateFile);
void writeDataToFile();

private:
rclcpp::CallbackGroup::SharedPtr callbackDepthPressure;
rclcpp::CallbackGroup::SharedPtr callbackIMU;
rclcpp::CallbackGroup::SharedPtr callbackClTool;
rclcpp::CallbackGroup::SharedPtr callbackManual;
rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    depth_pressure_sensor_subscription_;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr
    mag_subscription_;
rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr
    CLTool_subscription_;
rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr duration_subscription_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Manual_Control_sub;
rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr Manual_Override_sub;
std::ofstream &m_stateFile;

std::string depthPressureData_;
sensor_msgs::msg::Imu imuData_;
sensor_msgs::msg::MagneticField magData_;
std_msgs::msg::Int32MultiArray pwmData_;
int64_t durationData_;
bool manualControlData_;
bool manualOverrideTriggered_ = false;
rclcpp::TimerBase::SharedPtr timer_;

void depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg);
void imuSensorCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
void magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg);
void PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
void durationCallback(const std_msgs::msg::Int64::SharedPtr msg);
void ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg);
void ManualOverrideCallback(const std_msgs::msg::Empty::SharedPtr msg);
}
;
#endif // SENSORDATACONFIG_HPP
