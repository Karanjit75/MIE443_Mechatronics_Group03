#include "mie443_contest2/boxes.h"              // Loads the 5 scene object coordinates
#include "mie443_contest2/navigation.h"         // Sends navigation goals to Nav2
#include "mie443_contest2/robot_pose.h"         // Stores robot pose from /amcl_pose
#include "mie443_contest2/yoloInterface.h"      // Call camera and YOLO detection server
#include "mie443_contest2/arm_controller.h"     // Control arm and gripper motion
#include "mie443_contest2/apriltag_detector.h"  // For AprilTag detection

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <functional>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <sensor_msgs/msg/joint_state.hpp>


// Finite State Machine (FSM) states
enum class ContestState {
    Wait_for_AMCL_Pose,         // Wait until AMCL gives a valid pose
    Detect_Manipulable_Object,  // Detect the manipulable object on top of the robot
    Pick_Object,                // Pick manipulable object with the arm
    Visit_Scene_Objects,        // Go to the scene object locations one-by-one
    Drop_Object,                // Drop the object into the correct bin
    Return_Home,                // Return back to starting home location
    Done,                       // Finished successfully
    Failed                      // Something failed
};

// Stores one scene object detection
struct SceneDetection {
    int index;              // Which location was it at (0, 1, 2 ...)
    std::string name;       // Detected scene object
    float confidence;       // Confidence of YOLO detection
    double x;               // Location x
    double y;               // Location y
    double phi;             // Location phi
};

// Save results to required text file
void WriteResultsFile(const std::string& manipulable_name,
                      float manipulable_confidence,
                      const std::vector<SceneDetection>& scene_detections)
{
    std::ofstream myfile("contest2_results.txt");

    if (!myfile.is_open()) {
        std::cout << "Unable to open contest2_results.txt" << std::endl;
        return;
    }
    myfile << "Manipulable Object: " << manipulable_name << "\n";
    myfile << "Manipulable Confidence: " << manipulable_confidence << "\n";
    myfile << "Detected Scene Objects:\n";

    for (const auto& det : scene_detections) {
        // Output the object + location
        myfile << "Index " << det.index
               << " [x: " << det.x << ", y: " << det.y << ", phi: " << det.phi << "] "
               << "Object: " << det.name
               << " (confidence: " << det.confidence << ")\n";
    }
    myfile.close();
}

int main(int argc, char** argv)
{
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF as node parameters so MoveIt can build the arm model
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

    // Track whether AMCL callbacks are actually arriving
    bool amcl_received = false;

    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        [&](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
            amcl_received = true;
            robotPose.poseCallback(msg);
        }
    );

    // Load scene object coordinates
    Boxes boxes;
    if (!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        rclcpp::shutdown();
        return -1;
    }

    for (size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(),
                    "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // Initialize AprilTag Detection
    AprilTagDetector tagDetector(node);
    tagDetector.setReferenceFrame("arm_mount");
    std::vector<int> candidate_tags = {0, 1, 2, 3, 4};

    YoloInterface yoloDetector(node);   // Initialize YOLO Interface
    Navigation navigation(node);        // Initialize Nav2 stack
    ArmController armController(node);  // Initialize arm controller

    // --- NEW: Publisher for direct joint commands ---
    auto joint_cmd_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);

    // FSM start state
    ContestState state = ContestState::Wait_for_AMCL_Pose;

    // Start pose for returning home
    double start_x = 0.0;
    double start_y = 0.0;
    double start_phi = 0.0;
    bool start_pose_saved = false;

    // Manipulable object info
    std::string manipulable_object_name = "";
    float manipulable_object_confidence = 0.0f;

    // Scene object detections
    std::vector<SceneDetection> scene_detections;

    // Current scene-object location index
    size_t current_box_index = 0;

    // AMCL stability tracking
    int amcl_pose_samples = 0;
    const int required_amcl_samples = 10;

    // Match tracking
    int matching_box_index = -1;
    bool matching_object_found = false;

    // Contest-day manipulable object coordinates in arm base frame. Replace these with provided values
    double obj_x = 0.16;
    double obj_y = 0.00;
    double obj_z = 0.11;

    // Navigation retry settings
    int nav_attempts = 0;
    const int max_nav_attempts = 2;

    // Scene-object viewing retries
    const int max_scene_view_attempts = 3;
    const double scene_yaw_offsets[3] = {0.0, 0.20, -0.20};
    const float accepted_scene_confidence = 0.55f;

    // AprilTag search retries
    const int max_tag_view_attempts = 3;
    const double tag_yaw_offsets[3] = {0.0, 0.20, -0.20};

    // Detection retries for wrist camera
    const int max_wrist_detection_attempts = 3;

    // Contest timer
    auto contest_start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    while (rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        auto now = std::chrono::system_clock::now();
        secondsElapsed =
            std::chrono::duration_cast<std::chrono::seconds>(now - contest_start).count();

        switch (state) {
            case ContestState::Wait_for_AMCL_Pose:
            {
                // Only count samples if AMCL has actually published
                if (amcl_received && std::isfinite(robotPose.x) && std::isfinite(robotPose.y) && std::isfinite(robotPose.phi))
                {
                    amcl_pose_samples++;
                } else {
                    amcl_pose_samples = 0;
                }

                if (amcl_pose_samples >= required_amcl_samples) {
                    start_x = robotPose.x;
                    start_y = robotPose.y;
                    start_phi = robotPose.phi;
                    start_pose_saved = true;

                    RCLCPP_INFO(node->get_logger(),
                                "Start pose saved: x=%.3f, y=%.3f, phi=%.3f", start_x, start_y, start_phi);
                    state = ContestState::Detect_Manipulable_Object;
                }
                break;
            }

            case ContestState::Detect_Manipulable_Object:
            {
                RCLCPP_INFO(node->get_logger(), "Detecting manipulable object with Wrist Camera...");

                manipulable_object_name.clear();
                manipulable_object_confidence = 0.0f;

                // Retry a few times in case first detection misses
                for (int attempt = 0; attempt < max_wrist_detection_attempts; ++attempt) {
                    manipulable_object_name = yoloDetector.getObjectName(CameraSource::WRIST, true);

                    if (!manipulable_object_name.empty()) {
                        manipulable_object_confidence = yoloDetector.getConfidence();
                        break;
                    }

                    RCLCPP_WARN(node->get_logger(), "Wrist detection attempt %d/%d failed", attempt + 1, max_wrist_detection_attempts);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                }

                if (manipulable_object_name.empty()) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to detect manipulable object.");
                    state = ContestState::Visit_Scene_Objects;
                    break;
                }

                RCLCPP_INFO(node->get_logger(),"Manipulable object: %s (%.2f)", manipulable_object_name.c_str(), manipulable_object_confidence);
                state = ContestState::Pick_Object;
                break;
            }

            case ContestState::Pick_Object:
            {
                RCLCPP_INFO(node->get_logger(), "Executing instructor joint trajectory to move above object...");
                bool success = true;

                // Prepare the joint state message
                sensor_msgs::msg::JointState joint_msg;
                joint_msg.name = {"1", "2", "3", "4", "5", "6"};

                // Step 1 of trajectory (Directly above the manipulable object)
                joint_msg.position = {-1.5933, -0.8909, 0.8583, 1.600, 1.57, -0.0774};
                joint_cmd_pub->publish(joint_msg);
                std::this_thread::sleep_for(std::chrono::seconds(2)); // Give arm time to move

                if (manipulable_object_name.empty()) {
                    manipulable_object_name = yoloDetector.getObjectName(CameraSource::WRIST,true);
                }

                if (!manipulable_object_name.empty()) {
                    if (manipulable_confidence < 0.3) manipulable_confidence = yoloDetector.getConfidence();
                    if (manipulable_confidence > 0.3) {
            

                        // Step 2 open gripper
                        joint_msg.position = {-1.5933, -0.8909, 0.8583, 1.600, 1.57, 1};
                        joint_cmd_pub->publish(joint_msg);
                        std::this_thread::sleep_for(std::chrono::seconds(2)); // Give arm time to move

                
                        // Step 3 drop down
                        joint_msg.position = {-1.5933, -0.8909, 1.1, 1.3, 1.57, 1};
                        joint_cmd_pub->publish(joint_msg);
                        std::this_thread::sleep_for(std::chrono::seconds(2)); // Give arm time to move


                        // Step 4 of trajectory close gripper
                        joint_msg.position = {-1.5933, -0.8909, 1.1, 1.3, 1.57, 0.5};
                        joint_cmd_pub->publish(joint_msg);
                        std::this_thread::sleep_for(std::chrono::seconds(2));

                        // Step 5 go back up 
                        joint_msg.position = {-1.5933, -0.8909, 0.8583, 1.6, 1.57, 0.5};
                        joint_cmd_pub->publish(joint_msg);
                        std::this_thread::sleep_for(std::chrono::seconds(2));

                        // Step 6 Straighten arm out
                        joint_msg.position = {-1.5933, -0.8909, 0.8583, 0.2, 1.57, 0.5};
                        joint_cmd_pub->publish(joint_msg);
                        std::this_thread::sleep_for(std::chrono::seconds(2));

                RCLCPP_INFO(node->get_logger(), "Arm is in position. Proceeding with IK grab...");

                if (!success) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to pick up object using IK.");
                    //state = ContestState::Failed;
                    //break;
                }

                current_box_index = 0;
                matching_box_index = -1;
                matching_object_found = false;
                scene_detections.clear();

                state = ContestState::Visit_Scene_Objects;
                break;
            }

            /*
            case ContestState::Pick_Object:
            {
                RCLCPP_INFO(node->get_logger(), "Picking up object...");

                bool success = true;

                // Open gripper
                success &= armController.openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Approach from above
                success &= armController.moveToCartesianPose(
                    obj_x, obj_y, obj_z + 0.10, 0.0, M_PI/2, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Lower close to object
                success &= armController.moveToCartesianPose(
                    obj_x, obj_y, obj_z + 0.02, 0.0, M_PI/2, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Close gripper
                success &= armController.closeGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Lift object
                success &= armController.moveToCartesianPose(
                    obj_x, obj_y, obj_z + 0.12, 0.0, M_PI/2, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                if (!success) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to pick up object.");
                    state = ContestState::Failed;
                    break;
                }

                current_box_index = 0;
                matching_box_index = -1;
                matching_object_found = false;
                scene_detections.clear();

                state = ContestState::Visit_Scene_Objects;
                break;
            }
            */

            case ContestState::Visit_Scene_Objects:
            {
                // Once all 5 are visited, either drop or go home
                if (current_box_index >= boxes.coords.size()) {
                    RCLCPP_INFO(node->get_logger(), "Finished visiting all scene object locations.");

                    if (matching_object_found && matching_box_index >= 0) {
                        RCLCPP_INFO(node->get_logger(), "Matching object found at box %d.", matching_box_index);
                        state = ContestState::Drop_Object;
                    } else {
                        RCLCPP_WARN(node->get_logger(), "No matching scene object found. Returning home.");
                        state = ContestState::Return_Home;
                    }
                    break;
                }

                double goal_x = boxes.coords[current_box_index][0];
                double goal_y = boxes.coords[current_box_index][1];
                double goal_phi = boxes.coords[current_box_index][2];

                RCLCPP_INFO(node->get_logger(), "Navigating to box %zu: x=%.2f, y=%.2f, phi=%.2f", current_box_index, goal_x, goal_y, goal_phi);

                bool nav_success = navigation.moveToGoal(goal_x, goal_y, goal_phi);

                if (!nav_success) {
                    nav_attempts++;
                    RCLCPP_WARN(node->get_logger(), "Navigation failed for box %zu (attempt %d/%d)", current_box_index, nav_attempts, max_nav_attempts);

                    if (nav_attempts >= max_nav_attempts) {
                        RCLCPP_WARN(node->get_logger(), "Skipping box %zu after repeated nav failure", current_box_index);
                        nav_attempts = 0;
                        current_box_index++;
                    }
                    break;
                }

                nav_attempts = 0;
                std::this_thread::sleep_for(std::chrono::seconds(1));

                std::string best_detected_object = "";
                float best_conf = 0.0f;
                bool object_found = false;

                // --- NEW SEARCH STRATEGY PARAMETERS ---
                const int max_backup_steps = 4;        // Tries 0.0m, 0.15m, 0.30m, 0.45m back
                const double backup_increment = 0.15;  // Meters to back up each step
                const int rotation_steps = 8;          // 8 steps = 45 degrees per rotation
                const double rotation_increment = 2.0 * M_PI / rotation_steps;

                // Nested loop: gradually back up, and at each step, rotate 360 degrees
                for (int backup_step = 0; backup_step < max_backup_steps && !object_found; ++backup_step) {
                    
                    // Calculate backed-up position using the original goal orientation
                    // We subtract to move backwards along the approach vector
                    double current_backup_dist = backup_step * backup_increment;
                    double search_x = goal_x - (current_backup_dist * std::cos(goal_phi));
                    double search_y = goal_y - (current_backup_dist * std::sin(goal_phi));

                    for (int rot_step = 0; rot_step < rotation_steps && !object_found; ++rot_step) {
                        
                        // Calculate new rotation angle
                        double search_phi = goal_phi + (rot_step * rotation_increment);
                        
                        // Keep search_phi bounded between -PI and PI
                        search_phi = std::atan2(std::sin(search_phi), std::cos(search_phi));

                        // Only command navigation if we are actually moving/rotating away from the initial stop
                        if (backup_step > 0 || rot_step > 0) {
                            RCLCPP_INFO(node->get_logger(), "Searching box %zu: Backed up %.2fm, Angle %.2f rad", 
                                        current_box_index, current_backup_dist, search_phi);
                            
                            bool look_success = navigation.moveToGoal(search_x, search_y, search_phi);
                            if (!look_success) {
                                RCLCPP_WARN(node->get_logger(), "Failed to move to search pose, trying next.");
                                continue;
                            }
                            // Give camera time to stabilize and grab a non-blurry frame
                            std::this_thread::sleep_for(std::chrono::milliseconds(700));
                        }

                        RCLCPP_INFO(node->get_logger(), "Detecting scene object with OAK-D...");

                        std::string detected_object = yoloDetector.getObjectName(CameraSource::OAKD, false);

                        if (!detected_object.empty()) {
                            float conf = yoloDetector.getConfidence();

                            RCLCPP_INFO(node->get_logger(), "Detected: %s (%.2f)", detected_object.c_str(), conf);

                            if (conf > best_conf) {
                                best_conf = conf;
                                best_detected_object = detected_object;
                            }

                            // If we hit our confidence threshold, break out of all search loops
                            if (conf >= accepted_scene_confidence) {
                                object_found = true;
                                RCLCPP_INFO(node->get_logger(), "Object confidently found! Stopping search for this box.");
                                break; 
                            }
                        }
                    }
                }

                // Save best detection at this scene-object location
                if (!best_detected_object.empty()) {
                    // Update detection with the coordinates where we ACTUALLY found it (optional, but good for accuracy)
                    scene_detections.push_back({
                        static_cast<int>(current_box_index),
                        best_detected_object,
                        best_conf,
                        goal_x,     
                        goal_y,     
                        goal_phi    
                    });

                    RCLCPP_INFO(node->get_logger(), "Best detection at box %zu: %s (%.2f)", current_box_index, best_detected_object.c_str(), best_conf);

                    // Record first matching location, but still visit all 5
                    if (!matching_object_found && best_detected_object == manipulable_object_name) {
                        matching_object_found = true;
                        matching_box_index = static_cast<int>(current_box_index);

                        RCLCPP_INFO(node->get_logger(), "Match recorded at box %d. Continuing remaining boxes.", matching_box_index);
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(), "No reliable scene object detection at box %zu after full 360/backup search", current_box_index);
                }
                
                current_box_index++;
                break;
            }
            

            case ContestState::Drop_Object:
            {
                if (!matching_object_found ||
                    matching_box_index < 0 ||
                    matching_box_index >= static_cast<int>(boxes.coords.size()))
                {
                    RCLCPP_WARN(node->get_logger(), "No valid matching box stored. Returning home.");
                    state = ContestState::Return_Home;
                    break;
                }

                RCLCPP_INFO(node->get_logger(), "Navigating back to matching object at box %d before drop.", matching_box_index);

                double goal_x = boxes.coords[matching_box_index][0];
                double goal_y = boxes.coords[matching_box_index][1];
                double goal_phi = boxes.coords[matching_box_index][2];

                bool nav_success = navigation.moveToGoal(goal_x, goal_y, goal_phi);
                if (!nav_success) {
                    RCLCPP_WARN(node->get_logger(), "Failed to navigate back to matching box.");
                    state = ContestState::Return_Home;
                    break;
                }

                std::this_thread::sleep_for(std::chrono::seconds(1));
                RCLCPP_INFO(node->get_logger(), "Searching for AprilTag near matching object...");

                std::vector<int> visible_tags;
                bool tag_found = false;

                // Search for visible tag from a few nearby headings
                for (int view_try = 0; view_try < max_tag_view_attempts; ++view_try) {
                    double look_phi = goal_phi + tag_yaw_offsets[view_try];

                    if (view_try > 0) {
                        RCLCPP_INFO(node->get_logger(), "Repositioning to look for tag (try %d/%d)", view_try + 1, max_tag_view_attempts);

                        bool look_success = navigation.moveToGoal(goal_x, goal_y, look_phi);
                        if (!look_success) {
                            RCLCPP_WARN(node->get_logger(), "Could not reposition for tag search try %d", view_try + 1);
                            continue;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(700));
                    }

                    for (int i = 0; i < 6; ++i) {
                        rclcpp::spin_some(node);
                        visible_tags = tagDetector.getVisibleTags(candidate_tags);

                        if (!visible_tags.empty()) {
                            tag_found = true;
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(250));
                    }

                    if (tag_found) {
                        break;
                    }
                }

                if (!tag_found) {
                    RCLCPP_WARN(node->get_logger(),
                                "No tags visible after all search attempts.");
                    state = ContestState::Return_Home;
                    break;
                }

                int tag_id = visible_tags.front();

                geometry_msgs::msg::Pose tag_pose;
                bool pose_found = false;

                for (int i = 0; i < 5; ++i) {
                    auto pose = tagDetector.getTagPose(tag_id);

                    if (pose.has_value()) {
                        tag_pose = pose.value();
                        pose_found = true;
                        break;
                    }

                    rclcpp::spin_some(node);
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }

                if (!pose_found) {
                    RCLCPP_WARN(node->get_logger(), "Could not get tag pose after retries.");
                    state = ContestState::Return_Home;
                    break;
                }

                RCLCPP_INFO(node->get_logger(), "Using tag %d at position (%.3f, %.3f, %.3f)", tag_id, tag_pose.position.x, tag_pose.position.y, tag_pose.position.z);

                // This assumes getTagPose() returns coordinates usable by the arm controller. Must verify this on hardware. If frame is wrong, the arm will miss.
                double pre_x = tag_pose.position.x;
                double pre_y = tag_pose.position.y;
                double pre_z = tag_pose.position.z + 0.20;

                double drop_x = tag_pose.position.x;
                double drop_y = tag_pose.position.y;
                double drop_z = tag_pose.position.z + 0.10;

                bool success = true;

                // Move above bin
                success &= armController.moveToCartesianPose(
                    pre_x, pre_y, pre_z, 0.0, M_PI/2, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Lower toward bin
                success &= armController.moveToCartesianPose(
                    drop_x, drop_y, drop_z, 0.0, M_PI/2, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Release object
                success &= armController.openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));

                if (!success) {
                    RCLCPP_WARN(node->get_logger(), "Drop attempt failed.");
                }

                state = ContestState::Return_Home;
                break;
            }

            case ContestState::Return_Home:
            {
                if (start_pose_saved) {
                    RCLCPP_INFO(node->get_logger(), "Returning to start pose...");
                    bool home_success = navigation.moveToGoal(start_x, start_y, start_phi);

                    if (!home_success) {
                        RCLCPP_WARN(node->get_logger(), "Failed to return exactly to the start pose.");
                    }
                }

                WriteResultsFile(manipulable_object_name, manipulable_object_confidence, scene_detections);
                state = ContestState::Done;
                break;
            }

            case ContestState::Done:
            {
                RCLCPP_INFO(node->get_logger(), "Contest routine completed.");
                rclcpp::shutdown();
                return 0;
            }

            case ContestState::Failed:
            {
                RCLCPP_ERROR(node->get_logger(), "Contest routine failed.");
                WriteResultsFile(manipulable_object_name, manipulable_object_confidence, scene_detections);
                rclcpp::shutdown();
                return -1;
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
