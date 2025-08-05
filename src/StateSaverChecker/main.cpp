#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/bool.hpp>
#include <iostream>
#include <fstream>
#include <thread>
#include <chrono>
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr StateSaverStatusPub;
std::string getLastLineEfficient(const std::string& filename) {
    std::ifstream fs(filename, std::ios::ate); // Open at end
    if (!fs.is_open()) {
       std::cout << "not yet" << std::endl;
    }

    std::string lastLine;
    long long pos = fs.tellg(); // Get current position (end of file)

    // Handle empty file case
    if (pos == 0) {
        return "";
    }

    // Move back one character at a time until newline or beginning of file
    while (pos > 0) {
        fs.seekg(--pos);
        char c = fs.peek();
        if (c == '\n') {
            // Found newline, read the rest of the line
            fs.seekg(pos + 1); // Move past the newline
            std::getline(fs, lastLine);
            break;
        }else{
            std::cout << c << std::endl;
        }
    }

    // If no newline found (single line file or no trailing newline)
    if (lastLine.empty() && fs.tellg() == 0) {
        fs.seekg(0); // Go to beginning
        std::getline(fs, lastLine);
    }
    
    fs.close();
    return lastLine;
}

int main(){
	 std::string stateFilePath = "../../state.csv";
	while(true){
		std::string LastLine = getLastLineEfficient(stateFilePath);
        std::cout << LastLine << "\n";
		std::this_thread::sleep_for(std::chrono::milliseconds(800));
	}
}
