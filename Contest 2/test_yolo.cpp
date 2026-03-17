#include "mie443_contest2/yoloInterface.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("yolo_test_node");

    // Initialize only the YOLO Interface
    YoloInterface yoloDetector(node);
    
    RCLCPP_INFO(node->get_logger(), "YOLO Test Node Started. Querying cameras at 2Hz...");
    RCLCPP_INFO(node->get_logger(), "Press Ctrl+C to exit.");

    while (rclcpp::ok()) {
        // Essential to process any incoming ROS callbacks inside yoloDetector
        rclcpp::spin_some(node);

        // --- Test OAK-D Camera ---
        std::string oakd_obj = yoloDetector.getObjectName(CameraSource::OAKD, false);
        if (!oakd_obj.empty()) {
            float oakd_conf = yoloDetector.getConfidence();
            RCLCPP_INFO(node->get_logger(), "[OAK-D] Detected: %s (Conf: %.2f)", oakd_obj.c_str(), oakd_conf);
        }

        // --- Test Wrist Camera ---
        // Setting 'false' here so it doesn't force a crop/specific manipulable logic if we just want to see general detections
        std::string wrist_obj = yoloDetector.getObjectName(CameraSource::WRIST, false);
        if (!wrist_obj.empty()) {
            float wrist_conf = yoloDetector.getConfidence();
            RCLCPP_INFO(node->get_logger(), "[WRIST] Detected: %s (Conf: %.2f)", wrist_obj.c_str(), wrist_conf);
        }

        // Pause to avoid spamming the terminal and the YOLO server
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    }

    rclcpp::shutdown();
    return 0;
}