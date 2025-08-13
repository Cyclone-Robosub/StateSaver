#include "StateSaver.hpp"
#include <filesystem>
#include <chrono>

StateSaver::StateSaver() : Node("state_saver")
{
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, 
        std::bind(&StateSaver::imu_callback, this, std::placeholders::_1));
    
    mag_subscription_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "/mag", 10, 
        std::bind(&StateSaver::mag_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),  // 10ms = 100Hz
        std::bind(&StateSaver::timer_callback, this));
    pwm_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("sent_pwm_topic",10, 
        std::bind(&StateSaver::pwm_callback, this, std::placeholders::_1));

    position_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("position_topic",10, 
        std::bind(&StateSaver::position_callback, this, std::placeholders::_1));

    waypoint_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("waypoint_topic",10, 
        std::bind(&StateSaver::waypoint_callback, this, std::placeholders::_1));
    

     depth_pressure_sensor_subscription_ =
        this->create_subscription<std_msgs::msg::String>(
            "depthPressureSensorData", rclcpp::QoS(5),
            std::bind(&StateSaver::depthPressureSensorCallback,
                        this, std::placeholders::_1));

    
    
    // Initialize CSV file
    initialize_csv_file();
}

StateSaver::~StateSaver()
{
    if (imu_data_file_.is_open()) {
        imu_data_file_.close();
    }
}

void StateSaver::initialize_csv_file()
{
    std::filesystem::create_directories("data");

    // Generate filename with timestamp
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << "data/imu_data_" << std::put_time(std::localtime(&now_time_t), "%Y%m%d_%H%M%S") << ".csv";
    csv_filename_ = ss.str();

    imu_data_file_.open(csv_filename_, std::ios::out);
    if (imu_data_file_.is_open()) {
        imu_data_file_ << "timestamp," << "px, py, pz, pp, pq, pr, wx, wy, wz, wp, wq, wr" 
        
                      << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
                      << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z,"
                      << "magnetic_field_x,magnetic_field_y,magnetic_field_z," 
        
                      << "pwm_1,pwm_2,pwm_3,pwm_4,pwm_5,pwm_6,pwm_7,pwm_8,"
                      << "depth_pressure_sensor"
                      << std::endl;
        file_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "CSV file initialized: %s", csv_filename_.c_str());
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_filename_.c_str());
    }
}
void StateSaver::pwm_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg){
    int i = 0;
    for(int32_t value : msg->data){
         int setvalue = (int)value;
        pwm_array[i] = setvalue;
        ++i;
    }
}
void StateSaver::position_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    int i = 0;
    for(float value : msg->data){
        position_array[i] = value;
        ++i;
    }
}
void StateSaver::waypoint_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
    int i = 0;
    for(float value : msg->data){
        waypoint_array[i] = value;
        ++i;
    }
}

void StateSaver::depthPressureSensorCallback(const std_msgs::msg::String::SharedPtr msg){
    depth_pressure_msg = msg->data;
}
void StateSaver::imu_callback(const std::shared_ptr<const sensor_msgs::msg::Imu> &msg)
{
  angular_velocity_x = msg->angular_velocity.x;
  angular_velocity_y = msg->angular_velocity.y;
  angular_velocity_z = msg->angular_velocity.z;
  linear_acceleration_x = msg->linear_acceleration.x;
  linear_acceleration_y = msg->linear_acceleration.y;
  linear_acceleration_z = msg->linear_acceleration.z;

  std::cout << "IMU Data:" << std::endl;
  std::cout << "  Orientation: x=" << msg->orientation.x 
            << " y=" << msg->orientation.y 
            << " z=" << msg->orientation.z 
            << " w=" << msg->orientation.w << std::endl;
  std::cout << "  Angular Velocity: x=" << msg->angular_velocity.x 
            << " y=" << msg->angular_velocity.y 
            << " z=" << msg->angular_velocity.z << std::endl;
  std::cout << "  Linear Acceleration: x=" << msg->linear_acceleration.x 
            << " y=" << msg->linear_acceleration.y 
            << " z=" << msg->linear_acceleration.z << std::endl;
  std::cout << "---" << std::endl;
}

void StateSaver::mag_callback(const std::shared_ptr<const sensor_msgs::msg::MagneticField> &msg)
{
  mag_field_x = msg->magnetic_field.x;
  mag_field_y = msg->magnetic_field.y;
  mag_field_z = msg->magnetic_field.z;
}

void StateSaver::write_csv_line()
{
    // Get current timestamp
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    imu_data_file_ << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S") << "."
                  << std::setfill('0') << std::setw(3) << now_ms.count() << ","
                  << angular_velocity_x << ","
                  << angular_velocity_y << ","
                  << angular_velocity_z << ","
                  << linear_acceleration_x << ","
                  << linear_acceleration_y << ","
                  << linear_acceleration_z << ","
                  << mag_field_x << ","
                  << mag_field_y << ","
                  << mag_field_z << ",";

    for(auto i : position_array){ 
      imu_data_file_ << i << ",";
    }
    for(auto i : waypoint_array){ 
      imu_data_file_ << i << ",";
    }   
    for(auto i : pwm_array){ 
      imu_data_file_ << i << ",";
    }
    imu_data_file_ << depth_pressure_msg;
    imu_data_file_ << "\n";

}
std::string StateSaver::getCurrentDateTime() {
  time_t now = time(0);
  tm *localTime = localtime(&now);
  char buffer[80];
  strftime(buffer, sizeof(buffer), "%H:%M:%S", localTime);
  return std::string(buffer);
}

void StateSaver::write_data_to_csv()
{
    write_csv_line();
    imu_data_file_.flush();
}

void StateSaver::timer_callback()
{
    if (!file_initialized_) {
        return;
    }

    std::lock_guard<std::mutex> lock(imu_mutex);
    write_data_to_csv();
}

