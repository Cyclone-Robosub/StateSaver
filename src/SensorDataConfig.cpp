#include "SensorDataConfig.hpp"
#include <iomanip>

SensorsDataConfig::SensorsDataConfig(std::ofstream& outputStateFile)
          : Node("sensorsNode"), m_stateFile(outputStateFile) {
    callbackDepthPressure = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackIMU = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
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
    auto ManualToggleOptions = rclcpp::SubscriptionOptions();
    ManualToggleOptions.callback_group = callbackManual;
    auto ManualOverrideOptions = rclcpp::SubscriptionOptions();
    ManualOverrideOptions.callback_group = callbackManual;

    depth_pressure_sensor_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "depthPressureSensorData", rclcpp::QoS(5),
            std::bind(&SensorsDataConfig::depthPressureSensorCallback, this, std::placeholders::_1),
            depthPressureOptions);

    // Priority
    // Need to input IMU initialization with ROS.

    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu", rclcpp::QoS(5),
        std::bind(&SensorsDataConfig::imuSensorCallback, this, std::placeholders::_1),
        imuOptions);
    mag_subscription_ =
        this->create_subscription<sensor_msgs::msg::MagneticField>(
            "mag", rclcpp::QoS(5),
            std::bind(&SensorsDataConfig::magCallback, this, std::placeholders::_1),
            magOptions);

                //Please implement Kory's data as soon as possible
    /*did_ins_subscription =
    this->create_subscription<sensor_msgs::msg::MagneticField>(
        "mag", rclcpp::QoS(5),
        std::bind(&ExecutiveLoop::, mainLoopObject,
                    std::placeholders::_1),
        did_ins_Options);*/
    CLTool_subscription_ =
        this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "array_Cltool_topic", rclcpp::QoS(10),
            std::bind(&SensorsDataConfig::PWMArrayCallback, this, std::placeholders::_1),
            commandOptions);
    duration_subscription_ = this->create_subscription<std_msgs::msg::Int64>(
        "duration_Cltool_topic", rclcpp::QoS(10),
        std::bind(&SensorsDataConfig::durationCallback, this, std::placeholders::_1),
        durationOptions);
    Manual_Control_sub = this->create_subscription<std_msgs::msg::Bool>(
        "manual_toggle_switch", rclcpp::QoS(10),
        std::bind(&SensorsDataConfig::ManualControlCallback, this, std::placeholders::_1),
        ManualToggleOptions);
    Manual_Override_sub = this->create_subscription<std_msgs::msg::Empty>(
        "manualOverride", rclcpp::QoS(4),
        std::bind(&SensorsDataConfig::ManualOverrideCallback, this, std::placeholders::_1),
        ManualOverrideOptions);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SensorsDataConfig::writeDataToFile, this));
}

void SensorsDataConfig::depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg) {
    depthPressureData_ = msg->data;
}
void SensorsDataConfig::imuSensorCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    imuData_ = *msg;
}
void SensorsDataConfig::magCallback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
    magData_ = *msg;
}
void SensorsDataConfig::PWMArrayCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    pwmData_ = *msg;
}
void SensorsDataConfig::durationCallback(const std_msgs::msg::Int64::SharedPtr msg) {
    durationData_ = msg->data;
}
void SensorsDataConfig::ManualControlCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    manualControlData_ = msg->data;
}
void SensorsDataConfig::ManualOverrideCallback(const std_msgs::msg::Empty::SharedPtr msg) {
    manualOverrideTriggered_ = true;
}

void SensorsDataConfig::writeDataToFile() {
    auto now = std::chrono::system_clock::now();
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
    m_stateFile << std::put_time(std::localtime(&currentTime), "%F %T") << ","
                     << "Manual Override Triggered" << std::endl;
    m_stateFile << std::put_time(std::localtime(&currentTime), "%F %T") << ",";
    m_stateFile << depthPressureData_ << ",";

    m_stateFile << imuData_.orientation.x << "," << imuData_.orientation.y << ","
                     << imuData_.orientation.z << "," << imuData_.orientation.w << ",";

    for (size_t i = 0; i < pwmData_.data.size(); ++i) {
        m_stateFile << pwmData_.data[i] << (i == pwmData_.data.size() - 1 ? "" : ",");
    }
    m_stateFile << ",";
    m_stateFile << durationData_ << ",";
    m_stateFile << manualControlData_ << ",";
    m_stateFile << (manualOverrideTriggered_ ? "true" : "false") << std::endl;

    manualOverrideTriggered_ = false;
	
}


