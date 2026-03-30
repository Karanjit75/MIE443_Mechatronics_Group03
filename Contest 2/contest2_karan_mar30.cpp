#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <cmath>
#include <set>
#include <vector>
#include <string>
#include <algorithm>

using namespace std::chrono_literals;

static void publishJointCommand(
    const rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr& pub,
    const std::vector<double>& q)
{
    sensor_msgs::msg::JointState msg;
    msg.name = {"1", "2", "3", "4", "5", "6"};
    msg.position = q;
    pub->publish(msg);
}

static std::string normalizeObjectName(const std::string& raw)
{
    std::string s = raw;
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

static void stopBase(const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& cmd_pub)
{
    geometry_msgs::msg::Twist cmd;
    cmd_pub->publish(cmd);
}

static void driveBaseFor(
    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& cmd_pub,
    double linear_x,
    double angular_z,
    double duration_s)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear_x;
    cmd.angular.z = angular_z;

    auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < duration_s) {
        cmd_pub->publish(cmd);
        std::this_thread::sleep_for(100ms);
    }

    stopBase(cmd_pub);
    std::this_thread::sleep_for(300ms);
}

static bool tryDetectAtCurrentView(YoloInterface& yoloDetector, const std::set<std::string>& valid_objects,
    std::string& detected_item, float& confidence)
{
    std::string detected_raw = yoloDetector.getObjectName(CameraSource::OAKD, false);
    detected_item = normalizeObjectName(detected_raw);
    confidence = yoloDetector.getConfidence();

    return !detected_item.empty() && valid_objects.count(detected_item) && confidence > 0.30f;
}

static bool tryPanningScan(Navigation& navigation, const std::vector<double>& base_goal,
    YoloInterface& yoloDetector, const std::set<std::string>& valid_objects,
    std::string& detected_item, float& confidence)
{
    std::vector<double> pan_offsets = {0.0, -M_PI / 6.0, M_PI / 6.0, -M_PI / 3.0, M_PI / 3.0, -M_PI / 2.0, M_PI / 2.0};

    for (double off : pan_offsets) {
        bool ok = navigation.moveToGoal(base_goal[0], base_goal[1], base_goal[2] + off);
        if (!ok) {
            continue;
        }

        std::this_thread::sleep_for(500ms);

        if (tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence)) {
            return true;
        }
    }

    return false;
}

static bool tryCloserRetry(Navigation& navigation, const std::vector<double>& base_goal,
    YoloInterface& yoloDetector, const std::set<std::string>& valid_objects,
    std::string& detected_item, float& confidence)
{
    double closer_dist = 0.18;
    double theta = base_goal[2];
    std::vector<double> closer_goal = {
        base_goal[0] + closer_dist * std::cos(theta),
        base_goal[1] + closer_dist * std::sin(theta),
        base_goal[2]
    };

    bool ok = navigation.moveToGoal(closer_goal[0], closer_goal[1], closer_goal[2]);
    if (!ok) {
        return false;
    }

    std::this_thread::sleep_for(500ms);

    if (tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence)) {
        return true;
    }

    return tryPanningScan(navigation, closer_goal, yoloDetector, valid_objects, detected_item, confidence);
}

static bool tryLocalScanManeuver(
    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& cmd_pub,
    YoloInterface& yoloDetector,
    const std::set<std::string>& valid_objects,
    std::string& detected_item,
    float& confidence)
{
    driveBaseFor(cmd_pub, -0.08, 0.0, 1.2);

    if (tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence)) {
        return true;
    }

    driveBaseFor(cmd_pub, 0.0, 0.45, 1.0);
    if (tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence)) {
        return true;
    }

    driveBaseFor(cmd_pub, 0.0, -0.9, 2.0);
    if (tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence)) {
        return true;
    }

    driveBaseFor(cmd_pub, 0.0, 0.45, 1.0);
    if (tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence)) {
        return true;
    }

    driveBaseFor(cmd_pub, 0.06, 0.0, 0.8);
    return tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence);
}

static int matchedCoordinateIndexToTagId(int coordinate_index)
{
    return coordinate_index - 1;
}

static bool alignBaseToTag(
    Navigation& navigation,
    const RobotPose& robotPose,
    const geometry_msgs::msg::Pose& tag_pose,
    rclcpp::Logger logger)
{
    double desired_x = 0.35;
    double desired_y = 0.00;

    double error_x = tag_pose.position.x - desired_x;
    double error_y = tag_pose.position.y - desired_y;

    double goal_x = robotPose.x + error_x * std::cos(robotPose.phi) - error_y * std::sin(robotPose.phi);
    double goal_y = robotPose.y + error_x * std::sin(robotPose.phi) + error_y * std::cos(robotPose.phi);
    double goal_phi = robotPose.phi;

    RCLCPP_INFO(logger, "Aligning to tag: rel=(%.3f, %.3f), global goal=(%.3f, %.3f, %.3f)", tag_pose.position.x, tag_pose.position.y, goal_x, goal_y, goal_phi);

    return navigation.moveToGoal(goal_x, goal_y, goal_phi);
}

static void localTagNudge(
    const rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr& cmd_pub,
    const geometry_msgs::msg::Pose& tag_pose)
{
    double y = tag_pose.position.y;
    double x = tag_pose.position.x;

    if (y > 0.04) driveBaseFor(cmd_pub, 0.0, 0.25, 0.5);
    else if (y < -0.04) driveBaseFor(cmd_pub, 0.0, -0.25, 0.5);

    if (x > 0.42) driveBaseFor(cmd_pub, 0.05, 0.0, 0.8);
    else if (x < 0.28) driveBaseFor(cmd_pub, -0.05, 0.0, 0.8);
}

int main(int argc, char** argv) {
    // Setup ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    // Load the arm URDF and SRDF directly as node parameters
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
    if (!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for (size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(node->get_logger(), "Box %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
                    i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    YoloInterface yoloDetector(node);
    Navigation navigation(node);
    ArmController armController(node);
    AprilTagDetector tagDetector(node, "tag36h11:", "base_link");

    auto joint_cmd_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);
    auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Let AMCL update before storing start pose
    for (int i = 0; i < 10; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(50ms);
    }

    std::vector<double> start_pose = {robotPose.x, robotPose.y, robotPose.phi};

    // Contest countdown timer
    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    enum class RobotState {
        SCAN_MANIPULABLE_OBJECT, PICK_MANIPULABLE_OBJECT, NAVIGATE_TO_SCENE,
        DETECT_OBJECT, ROTATE_SLIGHTLY, LOCATE_DISCARD, DROP_OBJECT,
        RETURN_TO_START, DONE, FAILED
    };

    RobotState current_state = RobotState::SCAN_MANIPULABLE_OBJECT;

    std::set<std::string> valid_objects = {"bottle", "potted plant", "motorcycle", "clock", "cup"};

    std::string manipulable_object_name = "cup";
    float manipulable_confidence = 1.0f;
    bool object_picked = false;

    std::vector<std::string> discovered_scene_objects;
    std::vector<float> discovered_scene_confidences;
    std::vector<int> discovered_coordinate_indices;

    int matched_scene_coordinate_index = -1;
    int target_bin_tag_id = -1;
    geometry_msgs::msg::Pose target_bin_tag_pose;
    bool target_bin_tag_pose_valid = false;

    int current_target_index = 0;
    int total_targets = static_cast<int>(boxes.coords.size());
    int rotate_step_count = 0;

    // Execute strategy
    while (rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        auto now = std::chrono::system_clock::now();
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        switch (current_state) {

            case RobotState::SCAN_MANIPULABLE_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "Moving arm above pickup object and checking wrist camera.");

                publishJointCommand(joint_cmd_pub, {-1.5933, -0.8909, 0.8583, 1.600, 1.57, -0.0774});
                std::this_thread::sleep_for(2s);

                std::string detected_raw = yoloDetector.getObjectName(CameraSource::WRIST, true);
                std::string detected = normalizeObjectName(detected_raw);
                float conf = yoloDetector.getConfidence();

                if (!detected.empty() && valid_objects.count(detected) && conf > 0.30f) {
                    manipulable_confidence = conf;
                    RCLCPP_INFO(node->get_logger(), "Detected manipulable object: %s (%.2f)",
                                detected.c_str(), manipulable_confidence);
                    current_state = RobotState::PICK_MANIPULABLE_OBJECT;
                    break;
                }
                RCLCPP_WARN(node->get_logger(), "Primary wrist detection failed. Trying wrist sweep.");

                bool found = false;
                for (int i = -3; i <= 3; ++i) {
                    publishJointCommand(joint_cmd_pub, {-1.5933, -0.8909, 0.8583, 1.600 + 0.08 * i, 1.57 + 0.05 * i, -0.0774});
                    std::this_thread::sleep_for(2s);

                    detected_raw = yoloDetector.getObjectName(CameraSource::WRIST, true);
                    detected = normalizeObjectName(detected_raw);
                    conf = yoloDetector.getConfidence();

                    if (!detected.empty() && valid_objects.count(detected) && conf > 0.30f) {
                        manipulable_confidence = conf;
                        found = true;
                        RCLCPP_INFO(node->get_logger(), "Detected manipulable object using sweep: %s (%.2f)", detected.c_str(), manipulable_confidence);
                        break;
                    }
                }

                if (!found) {
                    RCLCPP_WARN(node->get_logger(), "Could not detect manipulable object from wrist camera. Continuing with known manipulable object cup.");
                }
                current_state = RobotState::PICK_MANIPULABLE_OBJECT;
                break;
            }

            case RobotState::PICK_MANIPULABLE_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "Executing pickup sequence...");
                    // Open gripper
                publishJointCommand(joint_cmd_pub, {-1.7933, -0.8909, 0.8583, 1.600, 1.57, -0.0774});
                std::this_thread::sleep_for(2s);
                    // Step 1 of trajectory (drop down to object)
                publishJointCommand(joint_cmd_pub, {-1.7933, -0.8909, 0.8583, 1.6, 1.57, 0.7});
                std::this_thread::sleep_for(2s);
                    // Close gripper
                publishJointCommand(joint_cmd_pub, {-1.7933, -0.8909, 1.1, 1.3, 1.57, 0.7});
                std::this_thread::sleep_for(2s);
                    // Step 2 of trajectory (Lift up)
                publishJointCommand(joint_cmd_pub, {-1.7933, -0.8909, 1.1, 1.6, 1.57, -0.12});
                std::this_thread::sleep_for(2s);
                    // Step 3 of trajectory (Carry pose)
                publishJointCommand(joint_cmd_pub, {0, -0.8909, 0.8583, 0.2, 1.57, -0.12});
                std::this_thread::sleep_for(2s);

                object_picked = true;
                current_state = RobotState::NAVIGATE_TO_SCENE;
                break;
            }

            case RobotState::NAVIGATE_TO_SCENE: {
                if (!object_picked) {
                    current_state = RobotState::SCAN_MANIPULABLE_OBJECT;
                    break;
                }

                if (current_target_index >= total_targets) {
                    current_state = RobotState::RETURN_TO_START;
                    break;
                }
                
                // --- STANDOFF CALCULATION UPDATE ---
                double obj_x = boxes.coords[current_target_index][0];
                double obj_y = boxes.coords[current_target_index][1];
                double obj_phi = boxes.coords[current_target_index][2];

                double standoff_dist = 0.5; // Stop 0.5m in front of the object
                double goal_x = obj_x + standoff_dist * std::cos(obj_phi);
                double goal_y = obj_y + standoff_dist * std::sin(obj_phi);
                
                double goal_phi = obj_phi + M_PI; // Turn around to face it
                
                // Normalize angle
                while (goal_phi > M_PI) goal_phi -= 2.0 * M_PI;
                while (goal_phi < -M_PI) goal_phi += 2.0 * M_PI;

                RCLCPP_INFO(node->get_logger(), "Navigating to standoff location %d...", current_target_index + 1);

                bool reached_goal = navigation.moveToGoal(goal_x, goal_y, goal_phi);

                if (reached_goal) {
                    current_state = RobotState::DETECT_OBJECT;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Failed to reach location %d, skipping.", current_target_index + 1);
                    current_target_index++;
                }
                break;
            }

            case RobotState::DETECT_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "Running OAK-D detection...");

                std::string detected_item;
                float confidence = 0.0f;
                
                // --- STANDOFF CALCULATION UPDATE ---
                double obj_x = boxes.coords[current_target_index][0];
                double obj_y = boxes.coords[current_target_index][1];
                double obj_phi = boxes.coords[current_target_index][2];

                double standoff_dist = 0.5;
                double base_x = obj_x + standoff_dist * std::cos(obj_phi);
                double base_y = obj_y + standoff_dist * std::sin(obj_phi);
                double base_phi = obj_phi + M_PI;

                while (base_phi > M_PI) base_phi -= 2.0 * M_PI;
                while (base_phi < -M_PI) base_phi += 2.0 * M_PI;

                std::vector<double> base_goal = {base_x, base_y, base_phi};

                if (tryDetectAtCurrentView(yoloDetector, valid_objects, detected_item, confidence)) {
                    RCLCPP_INFO(node->get_logger(), "Detected: %s (%.2f)", detected_item.c_str(), confidence);

                    discovered_scene_objects.push_back(detected_item);
                    discovered_scene_confidences.push_back(confidence);
                    discovered_coordinate_indices.push_back(current_target_index + 1);

                    if (detected_item == manipulable_object_name) {
                        matched_scene_coordinate_index = current_target_index + 1;
                        target_bin_tag_id = matchedCoordinateIndexToTagId(matched_scene_coordinate_index);
                        current_state = RobotState::LOCATE_DISCARD;
                    } else {
                        current_target_index++;
                        current_state = RobotState::NAVIGATE_TO_SCENE;
                    }
                } else if (tryPanningScan(navigation, base_goal, yoloDetector, valid_objects, detected_item, confidence)) {
                    RCLCPP_INFO(node->get_logger(), "Detected after panning: %s (%.2f)", detected_item.c_str(), confidence);

                    discovered_scene_objects.push_back(detected_item);
                    discovered_scene_confidences.push_back(confidence);
                    discovered_coordinate_indices.push_back(current_target_index + 1);

                    if (detected_item == manipulable_object_name) {
                        matched_scene_coordinate_index = current_target_index + 1;
                        target_bin_tag_id = matchedCoordinateIndexToTagId(matched_scene_coordinate_index);
                        current_state = RobotState::LOCATE_DISCARD;
                    } else {
                        current_target_index++;
                        current_state = RobotState::NAVIGATE_TO_SCENE;
                    }
                } else if (tryCloserRetry(navigation, base_goal, yoloDetector, valid_objects, detected_item, confidence)) {
                    RCLCPP_INFO(node->get_logger(), "Detected after closer retry: %s (%.2f)", detected_item.c_str(), confidence);

                    discovered_scene_objects.push_back(detected_item);
                    discovered_scene_confidences.push_back(confidence);
                    discovered_coordinate_indices.push_back(current_target_index + 1);

                    if (detected_item == manipulable_object_name) {
                        matched_scene_coordinate_index = current_target_index + 1;
                        target_bin_tag_id = matchedCoordinateIndexToTagId(matched_scene_coordinate_index);
                        current_state = RobotState::LOCATE_DISCARD;
                    } else {
                        current_target_index++;
                        current_state = RobotState::NAVIGATE_TO_SCENE;
                    }
                } else if (tryLocalScanManeuver(cmd_vel_pub, yoloDetector, valid_objects, detected_item, confidence)) {
                    RCLCPP_INFO(node->get_logger(), "Detected after local maneuver: %s (%.2f)", detected_item.c_str(), confidence);

                    discovered_scene_objects.push_back(detected_item);
                    discovered_scene_confidences.push_back(confidence);
                    discovered_coordinate_indices.push_back(current_target_index + 1);

                    if (detected_item == manipulable_object_name) {
                        matched_scene_coordinate_index = current_target_index + 1;
                        target_bin_tag_id = matchedCoordinateIndexToTagId(matched_scene_coordinate_index);
                        current_state = RobotState::LOCATE_DISCARD;
                    } else {
                        current_target_index++;
                        current_state = RobotState::NAVIGATE_TO_SCENE;
                    }
                } else {
                    rotate_step_count = 0;
                    current_state = RobotState::ROTATE_SLIGHTLY;
                }
                break;
            }

            case RobotState::ROTATE_SLIGHTLY: {
                std::vector<double> rotate_offsets = {0.0, -M_PI / 6.0, M_PI / 6.0, -M_PI / 3.0, M_PI / 3.0, -M_PI / 2.0, M_PI / 2.0};

                if (rotate_step_count >= static_cast<int>(rotate_offsets.size())) {
                    RCLCPP_WARN(node->get_logger(), "No valid detection at location %d, skipping.", current_target_index + 1);
                    current_target_index++;
                    current_state = RobotState::NAVIGATE_TO_SCENE;
                    break;
                }

                // --- STANDOFF CALCULATION UPDATE ---
                double obj_x = boxes.coords[current_target_index][0];
                double obj_y = boxes.coords[current_target_index][1];
                double obj_phi = boxes.coords[current_target_index][2];

                double standoff_dist = 0.5;
                double base_x = obj_x + standoff_dist * std::cos(obj_phi);
                double base_y = obj_y + standoff_dist * std::sin(obj_phi);
                double base_phi = obj_phi + M_PI; 
                
                double goal_phi = base_phi + rotate_offsets[rotate_step_count];

                while (goal_phi > M_PI) goal_phi -= 2.0 * M_PI;
                while (goal_phi < -M_PI) goal_phi += 2.0 * M_PI;

                bool reached_goal = navigation.moveToGoal(base_x, base_y, goal_phi);

                if (reached_goal) {
                    std::string detected_raw = yoloDetector.getObjectName(CameraSource::OAKD, false);
                    std::string detected_item = normalizeObjectName(detected_raw);
                    float confidence = yoloDetector.getConfidence();

                    if (!detected_item.empty() && valid_objects.count(detected_item) && confidence > 0.30f) {
                        RCLCPP_INFO(node->get_logger(), "Detected after rotation: %s (%.2f)", detected_item.c_str(), confidence);

                        discovered_scene_objects.push_back(detected_item);
                        discovered_scene_confidences.push_back(confidence);
                        discovered_coordinate_indices.push_back(current_target_index + 1);

                        if (detected_item == manipulable_object_name) {
                            matched_scene_coordinate_index = current_target_index + 1;
                            target_bin_tag_id = matchedCoordinateIndexToTagId(matched_scene_coordinate_index);
                            current_state = RobotState::LOCATE_DISCARD;
                        } else {
                            current_target_index++;
                            current_state = RobotState::NAVIGATE_TO_SCENE;
                        }
                        break;
                    }
                }
                rotate_step_count++;
                break;
            }

            case RobotState::LOCATE_DISCARD: {
                RCLCPP_INFO(node->get_logger(), "Searching for discard bin with AprilTags...");

                if (target_bin_tag_id < 0) {
                    RCLCPP_ERROR(node->get_logger(), "No matched scene coordinate stored, cannot choose correct bin.");
                    current_state = RobotState::FAILED;
                    break;
                }
                std::vector<int> candidate_tags = {target_bin_tag_id};
                auto visible_tags = tagDetector.getVisibleTags(candidate_tags);

                if (visible_tags.empty()) {
                    RCLCPP_WARN(node->get_logger(), "Target bin tag %d not visible yet.", target_bin_tag_id);
                    driveBaseFor(cmd_vel_pub, 0.0, 0.25, 0.5);
                    std::this_thread::sleep_for(300ms);
                    break;
                }

                auto tag_pose = tagDetector.getTagPose(target_bin_tag_id);
                if (!tag_pose.has_value()) {
                    RCLCPP_WARN(node->get_logger(), "Target bin tag %d visible but pose unavailable.", target_bin_tag_id);
                    driveBaseFor(cmd_vel_pub, 0.0, -0.25, 0.5);
                    std::this_thread::sleep_for(300ms);
                    break;
                }

                target_bin_tag_pose = tag_pose.value();
                target_bin_tag_pose_valid = true;

                RCLCPP_INFO(node->get_logger(), "Found correct discard tag ID %d at rel pose x=%.3f y=%.3f z=%.3f", target_bin_tag_id, target_bin_tag_pose.position.x, target_bin_tag_pose.position.y, target_bin_tag_pose.position.z);

                bool aligned = alignBaseToTag(navigation, robotPose, target_bin_tag_pose, node->get_logger());
                if (!aligned) {
                    RCLCPP_WARN(node->get_logger(), "Could not align base to target tag, attempting drop anyway.");
                } else {
                    std::this_thread::sleep_for(500ms);
                    auto refreshed_pose = tagDetector.getTagPose(target_bin_tag_id);
                    if (refreshed_pose.has_value()) {
                        target_bin_tag_pose = refreshed_pose.value();
                        target_bin_tag_pose_valid = true;
                    }
                }

                if (target_bin_tag_pose_valid) {
                    localTagNudge(cmd_vel_pub, target_bin_tag_pose);
                    std::this_thread::sleep_for(300ms);
                    auto refreshed_pose_2 = tagDetector.getTagPose(target_bin_tag_id);
                    if (refreshed_pose_2.has_value()) {
                        target_bin_tag_pose = refreshed_pose_2.value();
                        target_bin_tag_pose_valid = true;
                    }
                }

                current_state = RobotState::DROP_OBJECT;
                break;
            }

            case RobotState::DROP_OBJECT: {
                RCLCPP_INFO(node->get_logger(), "Dropping object...");

                double lateral_offset = 0.0;
                if (target_bin_tag_pose_valid) {
                    lateral_offset = target_bin_tag_pose.position.y;
                }
                if (lateral_offset > 0.10) lateral_offset = 0.10;
                if (lateral_offset < -0.10) lateral_offset = -0.10;

                double joint1_adjust = -0.8 * lateral_offset;
                double joint4_drop = 0.85;
                double joint4_retract = 0.20;

                publishJointCommand(joint_cmd_pub, {-1.5933 + joint1_adjust, -0.8909, 0.8583, joint4_drop, 1.57, 0.5});
                std::this_thread::sleep_for(2s);

                publishJointCommand(joint_cmd_pub, {-1.5933 + joint1_adjust, -0.8909, 0.93, 0.95, 1.57, 1.0});
                std::this_thread::sleep_for(2s);

                publishJointCommand(joint_cmd_pub, {-1.5933 + joint1_adjust, -0.8909, 0.93, 0.95, 1.57, 0});
                std::this_thread::sleep_for(1s);

                publishJointCommand(joint_cmd_pub, {-1.5933 + joint1_adjust, -0.8909, 0.8583, joint4_drop, 1.57, -0.03});
                std::this_thread::sleep_for(2s);

                publishJointCommand(joint_cmd_pub, {0, -0.8909, 0.8583, joint4_retract, 1.57, -0.03});
                std::this_thread::sleep_for(2s);

                current_state = RobotState::RETURN_TO_START;
                break;
            }

            case RobotState::RETURN_TO_START: {
                RCLCPP_INFO(node->get_logger(), "Returning to start pose...");
                bool returned = navigation.moveToGoal(start_pose[0], start_pose[1], start_pose[2]);
                current_state = returned ? RobotState::DONE : RobotState::FAILED;
                break;
            }

            case RobotState::DONE: {
                RCLCPP_INFO(node->get_logger(), "Contest task completed.");
                secondsElapsed = 301;
                break;
            }

            case RobotState::FAILED: {
                RCLCPP_ERROR(node->get_logger(), "Contest task failed.");
                secondsElapsed = 301;
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (secondsElapsed > 300) {
        RCLCPP_WARN(node->get_logger(), "Contest time limit reached!");
    }

    std::ofstream outfile("contest2_results.txt");
    if (outfile.is_open()) {
        outfile << "=== MIE443 Contest 2 Results ===\n\n";
        outfile << "Manipulable Object: " << manipulable_object_name << "\n";
        outfile << "Confidence Score: " << manipulable_confidence << "\n\n";
        outfile << "=== Scene Objects Discovered ===\n";

        for (size_t i = 0; i < discovered_scene_objects.size(); ++i) {
            outfile << discovered_scene_objects[i]
                    << " | confidence=" << discovered_scene_confidences[i]
                    << " | coordinate_index=" << discovered_coordinate_indices[i]
                    << "\n";
        }
    }

    RCLCPP_INFO(node->get_logger(), "Contest 2 node shutting down");
    rclcpp::shutdown();
    return 0;
}