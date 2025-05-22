#include "rclcpp/rclcpp.hpp"
#include "StateSaver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateSaver>());
    rclcpp::shutdown();
    return 0;
}
