#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters so that
    // MoveGroupInterface builds the SO-ARM101 model
    {
        std::string desc_dir = ament_index_cpp::get_package_share_directory("lerobot_description");
        std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
        if (urdf_file.is_open()) {
            std::stringstream ss;
            ss << urdf_file.rdbuf();
            node->declare_parameter("robot_description", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
        }

        std::string moveit_dir = ament_index_cpp::get_package_share_directory("lerobot_moveit");
        std::ifstream srdf_file(moveit_dir + "/config/so101.srdf");
        if (srdf_file.is_open()) {
            std::stringstream ss;
            ss << srdf_file.rdbuf();
            node->declare_parameter("robot_description_semantic", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm SRDF file");
        }
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node started");

    // Robot pose object + subscriber
    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );

    // Initialize box coordinates
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // Initialize YOLO object detector
    YoloInterface yoloDetector(node);

    /*
    //Initialize arm controller
    ArmController armController(node);

    RCLCPP_INFO(node->get_logger(), "=== TESTING ARM CONTROL ===");
    RCLCPP_INFO(node->get_logger(), "Moving arm to a reachable pose...");
    // Test arm movement with pose 1
    bool success = armController.moveToCartesianPose(0.043, 0.199, 0.313, -0.471, -0.557, 0.564, -0.387);

    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
        // Test gripper
        RCLCPP_INFO(node->get_logger(), "Testing gripper...");
        armController.openGripper();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        armController.closeGripper();
        std::this_thread::sleep_for(std::chrono::seconds(2));

    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose may be unreachable");
    }

    // // Tset arm movement with pose 2
    success = armController.moveToCartesianPose(0.142, -0.064, 0.400, -0.418, -0.844, 0.238, -0.237);

    if(success) {
        RCLCPP_INFO(node->get_logger(), "Arm moved successfully!");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Arm movement failed - pose may be unreachable");
    }
    */

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        /***YOUR CODE HERE***/
        /***TEST CODE FOR YOLO DETECTION ***/
        static uint64_t lastYoloTime = 0;
        if (secondsElapsed >= lastYoloTime + 2) {
            lastYoloTime = secondsElapsed;

            RCLCPP_INFO(node->get_logger(), "--- YOLO Detection (OAK-D Camera) ---");
            std::string detected = yoloDetector.getObjectName(CameraSource::OAKD);

            if(!detected.empty()) {
                float confidence = yoloDetector.getConfidence();
                RCLCPP_INFO(node->get_logger(), "Detected: %s (Confidence: %.2f)", detected.c_str(), confidence);
            } else {
                RCLCPP_INFO(node->get_logger(), "No object detected");
            }
        }


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}
