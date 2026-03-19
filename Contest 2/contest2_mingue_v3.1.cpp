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
#include <cmath>

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

    // Initialize AprilTagDetector
    AprilTagDetector tagDetector(node, "tag36h11:", "base_link");

    // Initialize Arm Controller
    ArmController armController(node);

    // Initialize Navigation
    Navigation navigation(node);

    // Spin node to save initial pose data.
    for (int i = 0; i < 10; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    const std::vector<double> start_pose = {robotPose.x, robotPose.y, robotPose.phi};

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    // Define states for state machine:
    enum class RobotState {
        SCAN_MANIPULABLE_OBJECT,
        NAVIGATE_TO_SCENE,
        DETECT_OBJECT,
        ROTATE_SLIGHTLY,
        // CHANGE_PERSPECTIVE,
        LOCATE_DISCARD,
        RETURN_TO_START,
        DONE
    };
    // Valid bin object options
    std::set<std::string> valid_objects = {"waterbottle", "plant", "motorcycle", "piggybank", "coffee_cup"};

    // Define fixed relative position of manipulable object here:
    std::vector<double> manipulable_object_rel_pos = {0.095, 0.001, 0.180, -0.172, -0.018, 1.782};

    // Manipulable object data
    std::string manipulable_object_name = "";
    float manipulable_confidence = 0.0;
    
    // Found object data
    std::vector<std::string> discovered_scene_objects;
    std::vector<float> discovered_object_confidence;

    // Initialize tracking variables
    RobotState current_state = RobotState::NAVIGATE_TO_SCENE;
    int current_target_index = 0;
    int total_targets = boxes.coords.size();

    // Execute strategy
    while(rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        // Calculate elapsed time
        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        // State machine
        switch(current_state) {

            case RobotState::SCAN_MANIPULABLE_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "Scanning manipulable object on top plate...");
                // Manually add pose for manipulable object
                armController.openGripper();
                armController.moveToCartesianPose(manipulable_object_rel_pos[0], 
                                                    manipulable_object_rel_pos[1],
                                                    manipulable_object_rel_pos[2],
                                                    manipulable_object_rel_pos[3],
                                                    manipulable_object_rel_pos[4],
                                                    manipulable_object_rel_pos[5]);
                std::this_thread::sleep_for(std::chrono::seconds(2));

                for (int i = -3; i < 3; i++) {
                    manipulable_object_name = yoloDetector.getObjectName(CameraSource::WRIST, true);
                    if (!manipulable_object_name.empty()) {
                        manipulable_confidence = yoloDetector.getConfidence();
                        if (manipulable_confidence > 0.3) {
                            RCLCPP_INFO(node->get_logger(), "Found manipulable object: %s (Confidence: %.2f)", 
                                    manipulable_object_name.c_str(), manipulable_confidence);
                            armController.moveToCartesianPose(manipulable_object_rel_pos[0], 
                                    manipulable_object_rel_pos[1],
                                    manipulable_object_rel_pos[2],
                                    manipulable_object_rel_pos[3],
                                    manipulable_object_rel_pos[4],
                                    manipulable_object_rel_pos[5]);
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            armController.closeGripper();
                            current_state = RobotState::NAVIGATE_TO_SCENE;
                            break;
                        }
                    }
                    armController.moveToCartesianPose(manipulable_object_rel_pos[0],
                                                        manipulable_object_rel_pos[1],
                                                        manipulable_object_rel_pos[2],
                                                        manipulable_object_rel_pos[3]+0.05*i,
                                                        manipulable_object_rel_pos[4]+0.05*i,
                                                        manipulable_object_rel_pos[5]+0.05*i);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                current_state = RobotState::NAVIGATE_TO_SCENE;
                break;
            }

            case RobotState::NAVIGATE_TO_SCENE: {
                if (current_target_index < total_targets) {
                    // Extract coordinates from 2D vector
                    // x, y, and orientation at 0, 1, 2
                    double goal_x = boxes.coords[current_target_index][0];
                    double goal_y = boxes.coords[current_target_index][1];
                    double goal_phi = boxes.coords[current_target_index][2];

                    RCLCPP_INFO(node->get_logger(), "Navigating to scene object %d...", current_target_index + 1);

                    bool reached_goal = navigation.moveToGoal(goal_x, goal_y, goal_phi);

                    if (reached_goal) {
                        // Transition to detection state
                        current_state = RobotState::DETECT_OBJECT;
                    } else {
                        RCLCPP_WARN(node->get_logger(), "Failed to reach %d. Retrying or skipping.", current_target_index + 1);
                        current_state = RobotState::NAVIGATE_TO_SCENE;
                    }
                } else {
                    // All boxes visited
                    current_state = RobotState::RETURN_TO_START;
                }
                break;
            }

            case RobotState::DETECT_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "Running YOLO detection...");
                std::string detected_item = yoloDetector.getObjectName(CameraSource::OAKD, false);                
                if (!detected_item.empty()) {
                    float confidence = yoloDetector.getConfidence();
                    if (valid_objects.count(detected_item) && confidence > 0.3) {
                        RCLCPP_INFO(node->get_logger(), "Successfully Detected: %s (Confidence: %.2f)", detected_item.c_str(), confidence);
                        discovered_scene_objects.push_back(detected_item);
                        discovered_object_confidence.push_back(confidence);
                        if (detected_item == manipulable_object_name) {
                            current_state = RobotState::LOCATE_DISCARD;
                        } else {
                            current_target_index++;
                            current_state = RobotState::NAVIGATE_TO_SCENE;
                        }
                    }
                } else {
                    RCLCPP_INFO(node->get_logger(), "No object detected at this location, rotating robot.");
                    current_state = RobotState::ROTATE_SLIGHTLY;
                }
                break;
            }

            case RobotState::ROTATE_SLIGHTLY: {
                double goal_x = boxes.coords[current_target_index][0];
                double goal_y = boxes.coords[current_target_index][1];
                double goal_phi = boxes.coords[current_target_index][2];

                for (int i = 0; i < 12; ++i) {
                    bool rotate_goal = navigation.moveToGoal(goal_x, goal_y, goal_phi+((M_PI*i)/6));
                    if (rotate_goal) {
                        std::string detected_item = yoloDetector.getObjectName(CameraSource::OAKD, false);
                        if (!detected_item.empty()) {
                            float confidence = yoloDetector.getConfidence();
                            if (confidence > 0.3) {
                                RCLCPP_INFO(node->get_logger(), "Successfully Detected Something. Returning to Detect Object State with Current Angle.");
                                current_state = RobotState::DETECT_OBJECT;
                                break;
                            }
                        }
                    }
                }
                RCLCPP_WARN(node->get_logger(), "No object detected at location index %d.", current_target_index + 1);
                current_target_index++;
                current_state = RobotState::NAVIGATE_TO_SCENE;
                break;
            }

            // case RobotState::CHANGE_PERSPECTIVE: {
            //     RCLCPP_INFO(node->get_logger(), "Trying different perspectives...");
            //     std::vector<double> reference_coordinate = {boxes.coords[current_target_index][0], 
            //                                                         boxes.coords[current_target_index][1], 
            //                                                         boxes.coords[current_target_index][2]};
            //     bool back_up = navigation.moveToGoal(reference_coordinate[0]-0.5*cos(reference_coordinate[2]+M_PI), 
            //                                         reference_coordinate[1]-0.5*sin(reference_coordinate[2]+M_PI), 
            //                                         reference_coordinate[2]);
            //     if (back_up) {
            //         std::string detected_item = yoloDetector.getObjectName(CameraSource::OAKD, false);
            //         if (!detected_item.empty()) {
            //             float confidence = yoloDetector.getConfidence();
            //             if (/*valid_objects.count(detected_item) &&*/ confidence > 0.5) {
            //                 RCLCPP_INFO(node->get_logger(), "Successfully Detected: %s (Confidence: %.2f)", detected_item.c_str(), confidence);
            //                 discovered_scene_objects.push_back(detected_item);
            //                 discovered_object_confidence.push_back(confidence);
            //                 bool back = navigation.moveToGoal(reference_coordinate[0], reference_coordinate[1], reference_coordinate[2]);
            //                 if (back && (detected_item == manipulable_object_name)) {
            //                     current_state = RobotState::LOCATE_DISCARD;
            //                 }
            //             }
            //         }
            //     } else {
            //         current_target_index++;
            //         current_state = RobotState::NAVIGATE_TO_SCENE;
            //     }
            //     break;
            // }

            case RobotState::LOCATE_DISCARD: {
                RCLCPP_INFO(node->get_logger(), "Found matching bin, searching for bin tag...");
                
                // Possible tag IDs
                std::vector<int> possible_tags = {0, 1, 2, 3, 4};

                // List of tags visible to the camera
                std::vector<int> visible_tags = tagDetector.getVisibleTags(possible_tags);

                if (!visible_tags.empty()) {
                    int target_tag_id = visible_tags[0];
                    
                    // Find relative 3D pose of the tag compared to the base_link
                    auto tag_pose_opt = tagDetector.getTagPose(target_tag_id);

                    if (tag_pose_opt.has_value()) {
                        
                        // Extract bin coordinates.
                        geometry_msgs::msg::Pose tag_pose = tag_pose_opt.value();

                        double bin_x = tag_pose.position.x;
                        double bin_y = tag_pose.position.y;
                        double bin_z = tag_pose.position.z;
                        // TODO: figure out angle orientation (reference frame orientation?)
                        armController.moveToCartesianPose(bin_x, bin_y, bin_z + 0.15, 0, 0, 0);
                        armController.openGripper();
                        current_state = RobotState::RETURN_TO_START;
                    }
                }
                break;
            }

            case RobotState::RETURN_TO_START: {
                RCLCPP_INFO(node->get_logger(), "All targets visited. Returning to starting position...");
                // Return to starting pose
                bool returned = navigation.moveToGoal(start_pose[0], start_pose[1], start_pose[2]); 
                if (returned) {
                    current_state = RobotState::DONE;
                }
                break;
            }

            case RobotState::DONE: {
                RCLCPP_INFO(node->get_logger(), "Contest Sequence Complete!");
                // Force exit loop.
                secondsElapsed = 301;
                break;
            }
        }

        /***YOUR CODE HERE***/
        /***TEST CODE FOR YOLO DETECTION ***/
        // static uint64_t lastYoloTime = 0;
        // if (secondsElapsed >= lastYoloTime + 2) {
        //     lastYoloTime = secondsElapsed;

        //     RCLCPP_INFO(node->get_logger(), "--- YOLO Detection (OAK-D Camera) ---");
        //     std::string detected = yoloDetector.getObjectName(CameraSource::OAKD);

        //     if(!detected.empty()) {
        //         float confidence = yoloDetector.getConfidence();
        //         RCLCPP_INFO(node->get_logger(), "Detected: %s (Confidence: %.2f)", detected.c_str(), confidence);
        //     } else {
        //         RCLCPP_INFO(node->get_logger(), "No object detected");
        //     }
        // }


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }
    
    // Generate final results file.
    RCLCPP_INFO(node->get_logger(), "Generating final results file: contest2_results.txt");
    std::ofstream outfile("contest2_results.txt");
    if (outfile.is_open()) {
        outfile << "=== MIE443 Contest 2 Results ===\n\n";
        outfile << "Manipulable Object (On Robot): " << manipulable_object_name << "\n";
        outfile << "Confidence Score: " << manipulable_confidence << "\n\n";
        
        outfile << "=== Scene Objects Discovered ===\n";
        for (size_t i = 0; i < discovered_scene_objects.size(); ++i) {
            outfile << "Location " << i + 1 << " (Index " << i << "): " << discovered_scene_objects[i] << "\n";
        }
        outfile.close();
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}