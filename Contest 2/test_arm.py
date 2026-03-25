#include "mie443_contest2/arm_controller.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>
#include <cmath>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("test_arm");

    // Initialize arm controller and direct joint publisher
    ArmController armController(node);
    auto joint_cmd_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);

    // Give ROS 2 publishers a second to discover each other
    std::this_thread::sleep_for(std::chrono::seconds(1));

    RCLCPP_INFO(node->get_logger(), "Executing instructor joint trajectory...");
    
    sensor_msgs::msg::JointState joint_msg;
    joint_msg.name = {"1", "2", "3", "4", "5", "6"};

    // Step 1 of trajectory (Directly above the manipulable object)
    joint_msg.position = {-1.5933, -0.8909, 0.8583, 1.600, 2.7448, -0.0774};
    joint_cmd_pub->publish(joint_msg);
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Give arm time to move

    // Open gripper
    armController.openGripper();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Step 1 of trajectory (drop down to object)
    joint_msg.position = {-1.5933, -0.8909, 0.8583 - 0.1, 1.600, 2.7448, -0.0774};
    joint_cmd_pub->publish(joint_msg);
    std::this_thread::sleep_for(std::chrono::seconds(2)); // Give arm time to move

    // Close gripper
    armController.closeGripper();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Step 2 of trajectory
    joint_msg.position = {-1.5933, -0.58519, -0.23257, 1.6061, 2.7448, -0.0774};
    joint_cmd_pub->publish(joint_msg);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Step 3 of trajectory 
    joint_msg.position = {-0.4208, -0.5222, -0.5366, 0.8286, 2.7448, -0.0774};  
    joint_cmd_pub->publish(joint_msg);
    std::this_thread::sleep_for(std::chrono::seconds(2));


    
    RCLCPP_INFO(node->get_logger(), "Arm test complete.");
    rclcpp::shutdown();
    return 0;
}