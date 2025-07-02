#include "rclcpp/rclcpp.hpp"
#include "SensorDataConfig.hpp"
#include <iostream>
#include <fstream>
#include <filesystem> // Add this line to include the filesystem library

namespace fs = std::filesystem;

std::ofstream makeStateFile() {
    std::ofstream stateFile;
    fs::path stateFilePath = fs::current_path().parent_path().parent_path();
    std::string stateFileString = stateFilePath / "state.csv"; // Use / for path concatenation

    // Improved file handling: Create a new file each time.  You could alternatively implement a more sophisticated rotation strategy
    stateFile.open(stateFileString, std::ofstream::out); // Open for output, truncating if it exists

    if (stateFile.is_open()) {
        stateFile << "Time,Depth(m),Pressure,IMU Data,PWM Data" << std::endl;
        std::cout << "Created/Overwrote state file: " << stateFileString << std::endl;
    } else {
        std::cerr << "Error opening state file: " << stateFileString << std::endl;
    }
    return stateFile;
}

int main(int argc, char* argv[]) {
    std::ofstream output("output.txt"); // Declare output and error streams
    std::ofstream error("error.txt");

    if (!output.is_open() || !error.is_open()) {
        std::cerr << "Error opening output/error file" << std::endl;
        return 1;
    }

    std::ofstream stateFile = makeStateFile(); // Call the corrected makeStateFile

    if (!stateFile.is_open()) {
        std::cerr << "Error creating state file. Exiting." << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);
    auto sensorsNode = std::make_shared<SensorsDataConfig>(stateFile);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(sensorsNode);
    output << "ROS2 StateSaver running" << std::endl;
    executor.spin();
    rclcpp::shutdown();
	output << "Shutting Down" << std::endl;
    output.close();
    error.close();
    stateFile.close(); // Close the state file
    return 0;
}
