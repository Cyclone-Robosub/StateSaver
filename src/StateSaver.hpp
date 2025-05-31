#ifndef STATE_SAVER_HPP
#define STATE_SAVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include <mutex>
#include <fstream>
#include <chrono>
#include <iomanip>

class StateSaver : public rclcpp::Node
{
public:
    StateSaver();
    ~StateSaver();  // Add destructor to properly close the file

private:
    void imu_callback(const std::shared_ptr<const sensor_msgs::msg::Imu> &msg);
    void mag_callback(const std::shared_ptr<const sensor_msgs::msg::MagneticField> &msg);
    void pwm_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg);
    void timer_callback();
    void initialize_csv_file();  // New function to initialize CSV file
    void write_data_to_csv();
    void write_csv_line();
    std::string getCurrentDateTime();

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pwm_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr depth_pressure_sensor_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    // File handling
    std::ofstream imu_data_file_;
    std::string csv_filename_;
    bool file_initialized_ = false;

    // IMU data members
    std::mutex imu_mutex;
    double angular_velocity_x = 0.0;
    double angular_velocity_y = 0.0;
    double angular_velocity_z = 0.0;
    double linear_acceleration_x = 0.0;
    double linear_acceleration_y = 0.0;
    double linear_acceleration_z = 0.0;

    int pwm_array[8];
    std::string depth_pressure_msg;

    // Magnetic field data members
    double mag_field_x = 0.0;
    double mag_field_y = 0.0;
    double mag_field_z = 0.0;
};

#endif // STATE_SAVER_HPP
