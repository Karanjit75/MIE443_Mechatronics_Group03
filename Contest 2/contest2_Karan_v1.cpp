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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>

// Finite State Machine (FSM) States
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
    int index;          // which location was it at (0, 1, 2 ...)
    std::string name;   // detected scene object
    float confidence;   // confidence of YOLO Detection
};

// Create file to save results into a text file containing: 
    // - detected manipulable object name and corresponding confidence score
    // - a list of all detected scene objects and their corresponding locations (object + location)
void WriteResultsFile(const std::string& manipulable_name, float manipulable_confidence, const std::vector<SceneDetection>& scene_detections)
{
    std::ofstream myfile;                // create file stream
    myfile.open("contest2_results.txt"); // open file

    if (myfile.is_open())   // check if file is opened properly
    {
        myfile << "Manipulable Object: " << manipulable_name << "\n";
        myfile << "Manipulable Confidence: " << manipulable_confidence << "\n";
        myfile << "Detected Scene Objects:\n";

        for (const auto& det: scene_detections) // write each sense detection
        {
            myfile << "Index " << det.index
                   << ": " << det.name
                   << " (confidence: " << det.confidence << ")\n";
        }

        myfile.close(); // close file
    }
    else
    {
        std::cout << "Unable to open file" << std::endl;
    }
}

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

    // Initialize box coordinates (Load 5 scene object coordinates from coords.xml)
    Boxes boxes;
    if(!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    // Print the loaded scene object coordinates
    for(size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }
    
    // Create AprilTag detector
    AprilTagDetector tagDetector(node);
    std::vector<int> candidate_tags = {0, 1, 2, 3, 4}; // possible tag IDs on the bins

    // Initialize YOLO object detector
    YoloInterface yoloDetector(node);

    // Initialize Navigation controller
    Navigation navigation(node);

    // Initialize arm controller
    ArmController armController(node);

    // Start in the state where we wait for AMCL pose
    ContestState state = ContestState::Wait_for_AMCL_Pose;

    // Variables to store start pose so the robot can return back later
    double start_x = 0.0;
    double start_y = 0.0;
    double start_phi = 0.0;
    bool start_pose_saved = false;

    // Variables for manipulable object info
    std::string manipulable_object_name = "";
    float manipulable_object_confidence = 0.0f;

    // Store all scene object detections
    std::vector<SceneDetection> scene_detections;

    // Keep track of which scene objection location being visited currently
    size_t current_box_index = 0;

    // Try navigation once more before skipping a scene
    int nav_attempts = 0;
    const int max_nav_attempts = 2;

    // Try up to 3 viewing angles at each scene object
    const int max_scene_view_attempts = 3;
    const double scene_yaw_offsets[3] = {0.0, 0.20, -0.20};

    // If good confidence, stop early to save time (higher than YOLO detector conf = 0.50)
    const float accepted_scene_confidence = 0.55f;

    // For AprilTag detection, also try a few viewing angles
    const int max_tag_view_attempts = 3;
    const double tag_yaw_offsets[3] = {0.0, 0.20, -0.20};

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

        // Run code based on current contest state in the FSM
        switch (state) {
            // State 1: Wait for AMCL pose
            case ContestState::Wait_for_AMCL_Pose:
            {
                // Wait until pose is no longer zero
                if (std::abs(robotPose.x) > 1e-4 || std::abs(robotPose.y) > 1e-4 || std::abs(robotPose.phi) > 1e-4) {

                    // Save current pose as the start pose
                    start_x = robotPose.x;
                    start_y = robotPose.y;
                    start_phi = robotPose.phi;
                    start_pose_saved = true;

                    RCLCPP_INFO(node->get_logger(), "Start pose saved: x=%.3f, y=%.3f, phi=%.3f", start_x, start_y, start_phi);

                    // Move to State 2 (Detecting Manipulable Object)
                    state = ContestState::Detect_Manipulable_Object;
                }
                break;
            }

            // State 2: Detect manipulable object using wrist camera
            case ContestState::Detect_Manipulable_Object:
            {
                // Have to use Wrist Camera to detect the object on top plate of the Turtlebot
                RCLCPP_INFO(node->get_logger(), "Detecting manipulable object with Wrist Camera...");
                manipulable_object_name = yoloDetector.getObjectName(CameraSource::WRIST, true); // true saves annotated image as required

                // If detection failed, go to Failed State
                if (manipulable_object_name.empty()) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to detect manipulable object.");
                    state = ContestState::Failed;
                    break;
                }
                
                // Save confidence levels
                manipulable_object_confidence = yoloDetector.getConfidence();

                RCLCPP_INFO(node->get_logger(), "Manipulable object: %s (%.2f)", manipulable_object_name.c_str(), manipulable_object_confidence);

                // Move to State 3 (Pick Object)
                state = ContestState::Pick_Object;
                break;
            }

            // State 3: Pick object from top plate
            case ContestState::Pick_Object:
            {
                RCLCPP_INFO(node->get_logger(), "Picking up object...");

                // Manipulable Object Coordinates (replace these on contest day accordingly) - defined in (x,y,z)
                double obj_x = 0.16;
                double obj_y = 0.00;
                double obj_z = 0.11;

                    // Track whether all arm actions succeed
                bool success = true;    

                    // Open gripper before approaching the object
                success &= armController.openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));  

                    // Move arm above the object first (for easy gripping)
                success &= armController.moveToCartesianPose(obj_x, obj_y, obj_z + 0.10, 0.0, 1.57, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                    // Move arm down closer to the object
                success &= armController.moveToCartesianPose(obj_x, obj_y, obj_z + 0.02, 0.0, 1.57, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                    // Close gripper to grab the object
                success  &= armController.closeGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));

                    // Lift the object up 
                success &= armController.moveToCartesianPose(obj_x, obj_y, obj_z + 0.12, 0.0, 1.57, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));
                
                    // If any arm step fails, stop contest routine
                if (!success) {
                    RCLCPP_ERROR(node->get_logger(), "Failed to pick up object.");
                    state = ContestState::Failed;
                    break;
                }

                // Start from first scene object location
                current_box_index = 0;

                // Move to State 4 (Visit Scene Objects)
                state = ContestState::Visit_Scene_Objects;
                break;
            }

            // State 4: Visit scene object locations
            case ContestState::Visit_Scene_Objects:
            {
                // If all scene locations have been visited, return to home position
                if (current_box_index >= boxes.coords.size()) {
                    RCLCPP_INFO(node->get_logger(), "Visited all the scene object locations.");
                    state = ContestState::Return_Home;
                    break;
                }

                // Read current scene object goal pose
                double goal_x = boxes.coords[current_box_index][0];
                double goal_y = boxes.coords[current_box_index][1];
                double goal_phi = boxes.coords[current_box_index][2];

                RCLCPP_INFO(node->get_logger(), "Navigating to box %zu: x=%.2f, y=%.2f, phi=%.2f", current_box_index, goal_x, goal_y, goal_phi);

                // Navigate to this scene object
                bool nav_success = navigation.moveToGoal(goal_x, goal_y, goal_phi);
                    // If navigation fails, retry scene once more before skipping it
                if (!nav_success) {
                    nav_attempts++;

                    RCLCPP_WARN(node->get_logger(), "Navigation failed for box %zu (attempt %d)", current_box_index, nav_attempts);

                    if (nav_attempts >= max_nav_attempts) {
                        RCLCPP_WARN(node->get_logger(), "Skipping box %zu after repeated navigation failure", current_box_index);
                        nav_attempts = 0;
                        current_box_index++;
                    }
                    break;
                }

                // Reset nav_attempts after success
                nav_attempts = 0;
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Try 3 different small heading angles at this same scene objects for more data collection on confidence
                std::string best_detected_object = "";
                float best_conf = 0.0f;

                for (int view_try = 0; view_try < max_scene_view_attempts; view_try++) {
                    double look_phi = goal_phi + scene_yaw_offsets[view_try];

                    // First try to use the original angle, next try to rotate a little left/right for better view
                    if (view_try > 0) {
                        RCLCPP_INFO(node->get_logger(), "Repositioning for better view at box %zu (try %d)", current_box_index, view_try + 1);

                        bool look_success = navigation.moveToGoal(goal_x, goal_y, look_phi);

                        if (!look_success) {
                            RCLCPP_WARN(node->get_logger(), "Could not reposition for view %d at box %zu", view_try + 1, current_box_index);
                            continue;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(700));
                    }

                // Detect scene object using the OAK-D camera
                RCLCPP_INFO(node->get_logger(), "Detecting scene object with OAK-D (view %d/%d)...", view_try + 1, max_scene_view_attempts);
                std::string detected_object = yoloDetector.getObjectName(CameraSource::OAKD, false);

                // If something was detected, save it
                if (!detected_object.empty()) {
                    float conf = yoloDetector.getConfidence();

                    RCLCPP_INFO(node->get_logger(), "View %d detected: %s (%.2f)", view_try + 1, detected_object.c_str(), conf);

                    // Keep best detection across all views
                    if (conf > best_conf) {
                        best_conf = conf;
                        best_detected_object = detected_object;
                    }

                    // If confidence already good enough, stop early to save time
                    if (conf >= accepted_scene_confidence) {
                        break;
                    }
                } else {
                    RCLCPP_WARN(node->get_logger(), "No object detected on view %d at box %zu", view_try + 1, current_box_index);
                }
            }
            
            // Save best detection seen at this scene object
            if (!best_detected_object.empty()) {
                scene_detections.push_back({
                    static_cast<int>(current_box_index),
                    best_detected_object,
                    best_conf
                });

                RCLCPP_INFO(node->get_logger(), "Best detection at box %zu: %s (%.2f)", current_box_index, best_detected_object.c_str(), best_conf);

                // If best detected scene object matches the manipulable object, go to Drop_Object state
                if (best_detected_object == manipulable_object_name) {
                    RCLCPP_INFO(node->get_logger(), "Matching object found at box %zu", current_box_index);
                    state = ContestState::Drop_Object;
                    break;
                }
            } else {
                RCLCPP_WARN(node->get_logger(), "No reliable scene object detection at box %zu after all views", current_box_index);
            }

            // Move to next scene object location
            current_box_index++;
            break;
        }

            // State 5: Drop object into matching bin
            case ContestState::Drop_Object:
            {
                RCLCPP_INFO(node->get_logger(), "Searching for AprilTag near matching object...");

                // Use current matching scene pose as base pose for AprilTag search
                double goal_x = boxes.coords[current_box_index][0];
                double goal_y = boxes.coords[current_box_index][1];
                double goal_phi = boxes.coords[current_box_index][2];

                std::vector<int> visible_tags;
                bool tag_found = false;

                // Try few small viewing angles to look for the bin tag
                for (int view_try = 0; view_try < max_tag_view_attempts; view_try++) {
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

                    // Retry tag visibility a few times at this same view
                    for (int i = 0; i < 6; i++) {
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

                // If still no tag visible, give up and return home
                if (!tag_found) {
                    RCLCPP_WARN(node->get_logger(), "No tags visible after all tag search attempts");
                    state = ContestState::Return_Home;
                    break;
                }

                // Use first visible tag
                int tag_id = visible_tags.front();

                // Retry getting tag pose a few times
                geometry_msgs::msg::Pose tag_pose;
                bool pose_found = false;

                for (int i = 0; i < 5; i++) {
                    auto pose = tagDetector.getTagPose(tag_id);

                    if (pose.has_value()) {
                        tag_pose = pose.value();
                        pose_found = true;
                        break;
                    }

                    rclcpp::spin_some(node);
                    std::this_thread::sleep_for(std::chrono::milliseconds(250));
                }

                // If still no pose --> return home
                if (!pose_found) {
                    RCLCPP_WARN(node->get_logger(), "Could not get tag pose after retries.");
                    state = ContestState::Return_Home;
                    break;
                }

                RCLCPP_INFO(node->get_logger(), "Using tag %d at position (%.3f, %.3f, %.3f)", tag_id, tag_pose.position.x, tag_pose.position.y, tag_pose.position.z);

                // Offsets for moving above the bin first and then lowering into it (may need to tune these)
                double pre_x = tag_pose.position.x;
                double pre_y = tag_pose.position.y;
                double pre_z = tag_pose.position.z + 0.20;

                double drop_x = tag_pose.position.x;
                double drop_y = tag_pose.position.y;
                double drop_z = tag_pose.position.z + 0.10;

                bool success = true;

                // Move to a point above the bin
                success &= armController.moveToCartesianPose(pre_x, pre_y, pre_z, 0.0, 1.57, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Move lower towards the bin
                success &= armController.moveToCartesianPose(drop_x, drop_y, drop_z, 0.0, 1.57, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(1));

                // Open gripper to release object
                success &= armController.openGripper();
                std::this_thread::sleep_for(std::chrono::seconds(1));

                if (!success) {
                    RCLCPP_WARN(node->get_logger(), "Drop attempt failed.");
                }
                
                // After dropping object, return home
                state = ContestState::Return_Home;
                break;
            }

            // State 6: Return home
            case ContestState::Return_Home:
            {
                // If start pose was saved, go back there
                if (start_pose_saved) {
                    RCLCPP_INFO(node->get_logger(), "Returning to start pose...");
                    navigation.moveToGoal(start_x, start_y, start_phi);
                }

                // Save results to text file
                WriteResultsFile(manipulable_object_name, manipulable_object_confidence, scene_detections);

                // Finished
                state = ContestState::Done;
                break;
            }

            // State 7: Done
            case ContestState::Done:
            {
                RCLCPP_INFO(node->get_logger(), "Contest routine completed.");
                rclcpp::shutdown();
                return 0;
            }

            // State 8: Failed
            case ContestState::Failed:
            {
                RCLCPP_ERROR(node->get_logger(), "Contest routine failed.");
                // Still save any results collected till this point
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