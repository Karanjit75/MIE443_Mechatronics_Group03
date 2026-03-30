#include "mie443_contest2/boxes.h"
#include "mie443_contest2/navigation.h"
#include "mie443_contest2/robot_pose.h"
#include "mie443_contest2/yoloInterface.h"
#include "mie443_contest2/arm_controller.h"
#include "mie443_contest2/apriltag_detector.h"

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <thread>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <limits>
#include <array>
#include <algorithm>
#include <cctype>
#include <string>
#include <optional>
#include <cmath>

struct WristViewPose {
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
    std::string name;
};

struct DetectionResult {
    bool success = false;
    std::string class_name = "";
    float confidence = 0.0f;
    int best_view_index = -1;
};

struct SceneObservation {
    int index = -1;
    int tag_id = -1;  // ASSUMPTION: scene index maps to tag id
    double x = 0.0;
    double y = 0.0;
    double phi = 0.0;
    bool detected = false;
    std::string raw_name = "";
    std::string normalized_name = "";
    float confidence = 0.0f;
};

enum class ContestState {
    DETECT_MANIPULABLE_OBJECT,
    PICK_UP_OBJECT,
    VISIT_ALL_SCENES,
    RETURN_TO_START,
    SAVE_RESULTS,
    FINISHED
};

static void sleepAndSpin(const std::shared_ptr<rclcpp::Node>& node, int ms)
{
    auto start = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        auto now = std::chrono::steady_clock::now();
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
        if (elapsed >= ms) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

static std::string toLowerCopy(std::string s)
{
    std::transform(
        s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); }
    );
    return s;
}

static std::string normalizeObjectName(const std::string& raw_name)
{
    std::string s = toLowerCopy(raw_name);

    if (s == "cup") return "cup";
    if (s == "bottle") return "water bottle";
    if (s == "clock") return "clock";
    if (s == "potted plant" || s == "plant") return "potted plant";
    if (s == "motorcycle" || s == "motorbike") return "toy motorcycle";

    return s;
}

static bool isContestObject(const std::string& normalized_name)
{
    return normalized_name == "cup" ||
           normalized_name == "water bottle" ||
           normalized_name == "clock" ||
           normalized_name == "potted plant" ||
           normalized_name == "toy motorcycle";
}

static bool isCupMatch(const std::string& normalized_name, float confidence, float min_confidence = 0.50f)
{
    return (normalized_name == "cup" && confidence >= min_confidence);
}

static std::vector<WristViewPose> buildWristObservationPoses(
    double obj_x, double obj_y, double obj_z)
{
    std::vector<WristViewPose> poses;

    poses.push_back({
        obj_x + 0.02, obj_y + 0.00, obj_z + 0.14,
        -0.471, -0.557, 0.564, -0.387,
        "center"
    });

    poses.push_back({
        obj_x + 0.04, obj_y + 0.06, obj_z + 0.15,
        -0.471, -0.557, 0.564, -0.387,
        "left"
    });

    poses.push_back({
        obj_x + 0.04, obj_y - 0.06, obj_z + 0.15,
        -0.471, -0.557, 0.564, -0.387,
        "right"
    });

    poses.push_back({
        obj_x + 0.06, obj_y + 0.00, obj_z + 0.19,
        -0.471, -0.557, 0.564, -0.387,
        "high"
    });

    poses.push_back({
        obj_x - 0.01, obj_y + 0.00, obj_z + 0.12,
        -0.471, -0.557, 0.564, -0.387,
        "close"
    });

    return poses;
}

static DetectionResult detectObjectMultiView(
    const std::shared_ptr<rclcpp::Node>& node,
    ArmController& armController,
    YoloInterface& yoloDetector,
    const std::vector<WristViewPose>& poses)
{
    DetectionResult final_result;

    std::map<std::string, int> vote_count;
    std::map<std::string, float> best_conf_for_class;
    std::map<std::string, int> best_view_for_class;

    for (size_t i = 0; i < poses.size() && rclcpp::ok(); ++i) {
        const auto& p = poses[i];

        RCLCPP_INFO(
            node->get_logger(),
            "Moving to wrist observation pose %zu (%s): x=%.3f y=%.3f z=%.3f",
            i, p.name.c_str(), p.x, p.y, p.z
        );

        bool moved = armController.moveToCartesianPose(
            p.x, p.y, p.z,
            p.qx, p.qy, p.qz, p.qw
        );

        if (!moved) {
            RCLCPP_WARN(
                node->get_logger(),
                "Observation pose %zu (%s) unreachable. Skipping.",
                i, p.name.c_str()
            );
            continue;
        }

        sleepAndSpin(node, 1200);

        std::string detected = yoloDetector.getObjectName(CameraSource::WRIST, false);

        if (!detected.empty()) {
            float conf = yoloDetector.getConfidence();

            RCLCPP_INFO(
                node->get_logger(),
                "WRIST detection at pose %s: %s (%.2f)",
                p.name.c_str(), detected.c_str(), conf
            );

            vote_count[detected]++;

            if (best_conf_for_class.find(detected) == best_conf_for_class.end() ||
                conf > best_conf_for_class[detected]) {
                best_conf_for_class[detected] = conf;
                best_view_for_class[detected] = static_cast<int>(i);
            }
        } else {
            RCLCPP_INFO(
                node->get_logger(),
                "No detection from WRIST camera at pose %s",
                p.name.c_str()
            );
        }

        sleepAndSpin(node, 300);
    }

    if (vote_count.empty()) {
        RCLCPP_WARN(node->get_logger(), "Multi-view wrist scan found no object.");
        return final_result;
    }

    std::string winner = "";
    int winner_votes = -1;
    float winner_conf = -1.0f;

    for (const auto& kv : vote_count) {
        const std::string& cls = kv.first;
        int votes = kv.second;
        float conf = best_conf_for_class[cls];

        if (votes > winner_votes || (votes == winner_votes && conf > winner_conf)) {
            winner = cls;
            winner_votes = votes;
            winner_conf = conf;
        }
    }

    final_result.success = true;
    final_result.class_name = winner;
    final_result.confidence = winner_conf;
    final_result.best_view_index = best_view_for_class[winner];

    RCLCPP_INFO(
        node->get_logger(),
        "Multi-view result: %s selected with %d vote(s), best confidence %.2f",
        final_result.class_name.c_str(), winner_votes, final_result.confidence
    );

    return final_result;
}

static bool saveBestWristDetectionImage(
    const std::shared_ptr<rclcpp::Node>& node,
    ArmController& armController,
    YoloInterface& yoloDetector,
    const std::vector<WristViewPose>& poses,
    int best_view_index)
{
    if (best_view_index < 0 || best_view_index >= static_cast<int>(poses.size())) {
        RCLCPP_WARN(node->get_logger(), "Invalid best view index, cannot save image.");
        return false;
    }

    const auto& p = poses[best_view_index];

    RCLCPP_INFO(
        node->get_logger(),
        "Revisiting best wrist view (%s) to save annotated image.",
        p.name.c_str()
    );

    bool moved = armController.moveToCartesianPose(
        p.x, p.y, p.z,
        p.qx, p.qy, p.qz, p.qw
    );

    if (!moved) {
        RCLCPP_WARN(node->get_logger(), "Could not move back to best wrist view.");
        return false;
    }

    sleepAndSpin(node, 1200);

    std::string detected = yoloDetector.getObjectName(CameraSource::WRIST, true);

    if (detected.empty()) {
        RCLCPP_WARN(
            node->get_logger(),
            "Could not save annotated image because no detection was returned."
        );
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Annotated wrist-camera detection image saved.");
    return true;
}

static bool detectSingleSceneObject(
    const std::shared_ptr<rclcpp::Node>& node,
    YoloInterface& yoloDetector,
    SceneObservation& obs,
    bool save_image = false)
{
    sleepAndSpin(node, 1200);

    std::string detected = yoloDetector.getObjectName(CameraSource::OAKD, save_image);
    if (detected.empty()) {
        obs.detected = false;
        obs.raw_name = "";
        obs.normalized_name = "";
        obs.confidence = 0.0f;
        return false;
    }

    float conf = yoloDetector.getConfidence();
    std::string normalized = normalizeObjectName(detected);

    obs.detected = true;
    obs.raw_name = detected;
    obs.normalized_name = normalized;
    obs.confidence = conf;

    return true;
}

static bool detectSceneObjectMultiSample(
    const std::shared_ptr<rclcpp::Node>& node,
    YoloInterface& yoloDetector,
    SceneObservation& obs,
    int num_samples = 3)
{
    std::map<std::string, int> vote_count;
    std::map<std::string, float> best_conf;
    std::map<std::string, std::string> raw_name_for_class;

    for (int k = 0; k < num_samples && rclcpp::ok(); ++k) {
        SceneObservation temp = obs;
        bool ok = detectSingleSceneObject(node, yoloDetector, temp, false);

        if (ok && isContestObject(temp.normalized_name)) {
            vote_count[temp.normalized_name]++;
            raw_name_for_class[temp.normalized_name] = temp.raw_name;

            if (best_conf.find(temp.normalized_name) == best_conf.end() ||
                temp.confidence > best_conf[temp.normalized_name]) {
                best_conf[temp.normalized_name] = temp.confidence;
            }

            RCLCPP_INFO(
                node->get_logger(),
                "Sample %d at scene index %d: %s -> %s (%.2f)",
                k + 1, obs.index,
                temp.raw_name.c_str(),
                temp.normalized_name.c_str(),
                temp.confidence
            );
        } else {
            RCLCPP_INFO(
                node->get_logger(),
                "Sample %d at scene index %d: no valid contest object detected",
                k + 1, obs.index
            );
        }

        sleepAndSpin(node, 250);
    }

    if (vote_count.empty()) {
        obs.detected = false;
        obs.raw_name = "";
        obs.normalized_name = "";
        obs.confidence = 0.0f;
        return false;
    }

    std::string winner = "";
    int winner_votes = -1;
    float winner_conf = -1.0f;

    for (const auto& kv : vote_count) {
        const std::string& cls = kv.first;
        int votes = kv.second;
        float conf = best_conf[cls];

        if (votes > winner_votes || (votes == winner_votes && conf > winner_conf)) {
            winner = cls;
            winner_votes = votes;
            winner_conf = conf;
        }
    }

    obs.detected = true;
    obs.normalized_name = winner;
    obs.raw_name = raw_name_for_class[winner];
    obs.confidence = winner_conf;

    return true;
}

static bool writeContestResults(
    const std::shared_ptr<rclcpp::Node>& node,
    const std::string& manipulable_raw_name,
    const std::string& manipulable_normalized_name,
    float manipulable_confidence,
    const std::vector<SceneObservation>& scene_observations)
{
    std::ofstream out("contest2_results.txt");
    if (!out.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create contest2_results.txt");
        return false;
    }

    out << "Manipulable object:\n";
    out << "  raw_name: " << manipulable_raw_name << "\n";
    out << "  normalized_name: " << manipulable_normalized_name << "\n";
    out << "  confidence: " << manipulable_confidence << "\n\n";

    out << "Scene objects:\n";
    for (const auto& obs : scene_observations) {
        out << "  scene_index: " << obs.index << "\n";
        out << "    tag_id: " << obs.tag_id << "\n";
        out << "    map_pose: (" << obs.x << ", " << obs.y << ", " << obs.phi << ")\n";
        out << "    detected: " << (obs.detected ? "true" : "false") << "\n";
        out << "    raw_name: " << obs.raw_name << "\n";
        out << "    normalized_name: " << obs.normalized_name << "\n";
        out << "    confidence: " << obs.confidence << "\n";
    }

    out.close();
    RCLCPP_INFO(node->get_logger(), "Saved contest results to contest2_results.txt");
    return true;
}

static bool moveArmToTravelPose(ArmController& armController)
{
    return armController.moveToCartesianPose(
        0.142, -0.064, 0.400,
        -0.418, -0.844, 0.238, -0.237
    );
}

static bool openGripperSafe(ArmController& armController)
{
    armController.openGripper();
    return true;
}

static bool closeGripperSafe(ArmController& armController)
{
    armController.closeGripper();
    return true;
}

static bool pickUpManipulableObject(
    const std::shared_ptr<rclcpp::Node>& node,
    ArmController& armController,
    double obj_x, double obj_y, double obj_z)
{
    const double qx = -0.471;
    const double qy = -0.557;
    const double qz =  0.564;
    const double qw = -0.387;

    // EDIT THESE for grasp tuning
    const double pregrasp_z = obj_z + 0.16;  // EDIT THIS for pregrasp z
    const double grasp_z    = obj_z + 0.08;  // EDIT THIS for grasp z
    const double lift_z     = obj_z + 0.20;  // EDIT THIS for lift z

    RCLCPP_INFO(node->get_logger(), "Starting pickup sequence...");

    openGripperSafe(armController);
    sleepAndSpin(node, 800);

    bool ok = armController.moveToCartesianPose(obj_x, obj_y, pregrasp_z, qx, qy, qz, qw);
    if (!ok) {
        RCLCPP_WARN(node->get_logger(), "Failed to move to pregrasp pose.");
        return false;
    }
    sleepAndSpin(node, 1000);

    ok = armController.moveToCartesianPose(obj_x, obj_y, grasp_z, qx, qy, qz, qw);
    if (!ok) {
        RCLCPP_WARN(node->get_logger(), "Failed to descend to grasp pose.");
        return false;
    }
    sleepAndSpin(node, 1000);

    closeGripperSafe(armController);
    sleepAndSpin(node, 1000);

    ok = armController.moveToCartesianPose(obj_x, obj_y, lift_z, qx, qy, qz, qw);
    if (!ok) {
        RCLCPP_WARN(node->get_logger(), "Failed to lift object after grasp.");
        return false;
    }
    sleepAndSpin(node, 1000);

    ok = moveArmToTravelPose(armController);
    if (!ok) {
        RCLCPP_WARN(node->get_logger(), "Failed to move to travel pose after pickup.");
        return false;
    }

    RCLCPP_INFO(node->get_logger(), "Pickup sequence completed.");
    return true;
}

static bool moveArmToPreDropPose(ArmController& armController)
{
    // EDIT THIS for final drop pose tuning
    return armController.moveToCartesianPose(
        0.150, 0.000, 0.220,  // EDIT THESE for drop x,y,z
        -0.471, -0.557, 0.564, -0.387
    );
}

static bool dropObjectToBin(
    const std::shared_ptr<rclcpp::Node>& node,
    ArmController& armController)
{
    RCLCPP_INFO(node->get_logger(), "Executing drop sequence...");

    bool ok = moveArmToPreDropPose(armController);
    if (!ok) {
        RCLCPP_WARN(node->get_logger(), "Could not move to pre-drop pose.");
        return false;
    }

    sleepAndSpin(node, 1000);

    openGripperSafe(armController);
    sleepAndSpin(node, 1200);

    ok = moveArmToTravelPose(armController);
    if (!ok) {
        RCLCPP_WARN(node->get_logger(), "Could not return to travel pose after drop.");
    }

    return true;
}

static bool tryDetectSpecificTag(
    const std::shared_ptr<rclcpp::Node>& node,
    AprilTagDetector& tagDetector,
    int tag_id,
    int timeout_ms = 500)
{
    std::vector<int> visible = tagDetector.getVisibleTags({tag_id}, timeout_ms);
    return !visible.empty();
}

static bool localTagSearchAndDrop(
    const std::shared_ptr<rclcpp::Node>& node,
    Navigation& nav,
    AprilTagDetector& tagDetector,
    ArmController& armController,
    const SceneObservation& target_scene)
{
    // ASSUMPTION: tag_id == scene index
    const int target_tag = target_scene.tag_id;

    // Search poses around the matched scene/bin pose.
    // EDIT THESE offsets if tag visibility needs tuning.
    std::vector<std::array<double, 3>> search_goals = {
        {target_scene.x,          target_scene.y,          target_scene.phi},
        {target_scene.x,          target_scene.y,          target_scene.phi + 0.35},
        {target_scene.x,          target_scene.y,          target_scene.phi - 0.35},
        {target_scene.x - 0.20,   target_scene.y,          target_scene.phi},
        {target_scene.x,          target_scene.y + 0.20,   target_scene.phi},
        {target_scene.x,          target_scene.y - 0.20,   target_scene.phi}
    };

    for (size_t i = 0; i < search_goals.size() && rclcpp::ok(); ++i) {
        double gx = search_goals[i][0];
        double gy = search_goals[i][1];
        double gphi = search_goals[i][2];

        RCLCPP_INFO(node->get_logger(),
                    "Tag search pose %zu for tag %d -> x=%.2f y=%.2f phi=%.2f",
                    i, target_tag, gx, gy, gphi);

        bool nav_ok = nav.moveToGoal(gx, gy, gphi);
        if (!nav_ok) {
            RCLCPP_WARN(node->get_logger(), "Tag search navigation failed at pose %zu", i);
            continue;
        }

        sleepAndSpin(node, 1200);

        if (tryDetectSpecificTag(node, tagDetector, target_tag, 500)) {
            auto pose_opt = tagDetector.getTagPose(target_tag, 500);
            if (pose_opt.has_value()) {
                const auto& pose = pose_opt.value();
                RCLCPP_INFO(node->get_logger(),
                            "Detected tag %d in %s frame: pos(%.3f, %.3f, %.3f)",
                            target_tag,
                            tagDetector.getReferenceFrame().c_str(),
                            pose.position.x,
                            pose.position.y,
                            pose.position.z);

                // At this point we know the correct bin/tag is visible.
                // The actual arm drop pose still needs tuning relative to your real setup.
                return dropObjectToBin(node, armController);
            }
        }
    }

    RCLCPP_WARN(node->get_logger(), "Could not localize target tag %d during local search.", target_tag);
    return false;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("contest2");

    {
        std::string desc_dir =
            ament_index_cpp::get_package_share_directory("lerobot_description");
        std::ifstream urdf_file(desc_dir + "/urdf/so101.urdf");
        if (urdf_file.is_open()) {
            std::stringstream ss;
            ss << urdf_file.rdbuf();
            node->declare_parameter("robot_description", ss.str());
        } else {
            RCLCPP_ERROR(node->get_logger(), "Could not open arm URDF file");
        }

        std::string moveit_dir =
            ament_index_cpp::get_package_share_directory("lerobot_moveit");
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

    RobotPose robotPose(0, 0, 0);
    auto amclSub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose",
        10,
        std::bind(&RobotPose::poseCallback, &robotPose, std::placeholders::_1)
    );
    (void)amclSub;

    Boxes boxes;
    if (!boxes.load_coords()) {
        RCLCPP_ERROR(node->get_logger(), "ERROR: could not load box coordinates");
        return -1;
    }

    for (size_t i = 0; i < boxes.coords.size(); ++i) {
        RCLCPP_INFO(
            node->get_logger(),
            "Scene %zu coordinates: x=%.2f, y=%.2f, phi=%.2f",
            i, boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]
        );
    }

    YoloInterface yoloDetector(node);
    ArmController armController(node);
    Navigation nav(node);

    // Tutorial 4 initialization pattern
    AprilTagDetector tagDetector(node);
    std::vector<int> candidate_tags = {0, 1, 2, 3, 4};
    (void)candidate_tags;

    // Optional: use base_link or another frame if your detector supports it
    tagDetector.setReferenceFrame("base_link");

    sleepAndSpin(node, 2000);

    double start_x = robotPose.x;
    double start_y = robotPose.y;
    double start_phi = robotPose.phi;

    RCLCPP_INFO(
        node->get_logger(),
        "Recorded start pose: x=%.3f y=%.3f phi=%.3f",
        start_x, start_y, start_phi
    );

    // -------------------------------------------------------------------------
    // EDIT THIS: manipulable object center on robot top plate in ARM BASE frame
    // -------------------------------------------------------------------------
    double obj_x_arm = 0.12;   // EDIT THIS for object-on-robot x
    double obj_y_arm = 0.00;   // EDIT THIS for object-on-robot y
    double obj_z_arm = 0.07;   // EDIT THIS for object-on-robot z

    auto start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    RCLCPP_INFO(node->get_logger(), "Starting contest - 300 seconds timer begins now!");

    ContestState state = ContestState::DETECT_MANIPULABLE_OBJECT;

    bool wristScanDone = false;
    bool pickupDone = false;
    bool returnedToStart = false;
    bool resultsSaved = false;
    bool cupDropped = false;
    bool scenesVisited = false;

    std::string manipulable_raw_name = "";
    std::string manipulable_name = "";
    float manipulable_confidence = 0.0f;

    std::vector<SceneObservation> scene_observations;
    for (size_t i = 0; i < boxes.coords.size(); ++i) {
        SceneObservation obs;
        obs.index = static_cast<int>(i);
        obs.tag_id = static_cast<int>(i); // ASSUMPTION: bin tag ids are 0..4 matching scene index

        // ---------------------------------------------------------------------
        // EDIT THESE IN coords.xml for each scene/bin location:
        //   scene 0 x,y,phi
        //   scene 1 x,y,phi
        //   scene 2 x,y,phi
        //   scene 3 x,y,phi
        //   scene 4 x,y,phi
        // Example:
        //   plant coordinate -> edit corresponding coords.xml x,y,phi
        // ---------------------------------------------------------------------
        obs.x = boxes.coords[i][0];   // EDIT THIS in coords.xml for scene/bin x
        obs.y = boxes.coords[i][1];   // EDIT THIS in coords.xml for scene/bin y
        obs.phi = boxes.coords[i][2]; // EDIT THIS in coords.xml for scene/bin phi

        scene_observations.push_back(obs);
    }

    while (rclcpp::ok() && secondsElapsed <= 300) {
        rclcpp::spin_some(node);

        auto now = std::chrono::system_clock::now();
        secondsElapsed =
            std::chrono::duration_cast<std::chrono::seconds>(now - start).count();

        switch (state) {

        case ContestState::DETECT_MANIPULABLE_OBJECT:
        {
            if (!wristScanDone) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Phase 1: detect manipulable object on robot using wrist camera."
                );

                std::vector<WristViewPose> poses =
                    buildWristObservationPoses(obj_x_arm, obj_y_arm, obj_z_arm);

                DetectionResult result =
                    detectObjectMultiView(node, armController, yoloDetector, poses);

                if (result.success) {
                    manipulable_raw_name = result.class_name;
                    manipulable_name = normalizeObjectName(result.class_name);
                    manipulable_confidence = result.confidence;

                    RCLCPP_INFO(
                        node->get_logger(),
                        "Manipulable object detected: raw=%s normalized=%s confidence=%.2f",
                        manipulable_raw_name.c_str(),
                        manipulable_name.c_str(),
                        manipulable_confidence
                    );

                    saveBestWristDetectionImage(
                        node,
                        armController,
                        yoloDetector,
                        poses,
                        result.best_view_index
                    );

                    wristScanDone = true;
                    state = ContestState::PICK_UP_OBJECT;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Failed to detect manipulable object. Retrying...");
                }
            }
            break;
        }

        case ContestState::PICK_UP_OBJECT:
        {
            if (!pickupDone) {
                RCLCPP_INFO(node->get_logger(), "Phase 2: pick up manipulable object and hold it.");

                bool picked =
                    pickUpManipulableObject(node, armController, obj_x_arm, obj_y_arm, obj_z_arm);

                if (picked) {
                    pickupDone = true;
                    state = ContestState::VISIT_ALL_SCENES;
                } else {
                    RCLCPP_WARN(node->get_logger(), "Pickup failed. Retrying...");
                }
            }
            break;
        }

        case ContestState::VISIT_ALL_SCENES:
        {
            if (!scenesVisited) {
                RCLCPP_INFO(node->get_logger(), "Phase 3: visit all 5 scene coordinates.");

                for (size_t i = 0; i < scene_observations.size() && rclcpp::ok(); ++i) {
                    auto& obs = scene_observations[i];

                    RCLCPP_INFO(
                        node->get_logger(),
                        "Navigating to scene %d: x=%.2f y=%.2f phi=%.2f",
                        obs.index, obs.x, obs.y, obs.phi
                    );

                    bool nav_ok = nav.moveToGoal(obs.x, obs.y, obs.phi);
                    if (!nav_ok) {
                        RCLCPP_WARN(node->get_logger(), "Navigation failed for scene %d", obs.index);
                        obs.detected = false;
                        continue;
                    }

                    sleepAndSpin(node, 1500);

                    bool detected_ok = detectSceneObjectMultiSample(node, yoloDetector, obs, 3);

                    if (detected_ok) {
                        RCLCPP_INFO(
                            node->get_logger(),
                            "Scene %d detected: raw=%s normalized=%s confidence=%.2f",
                            obs.index,
                            obs.raw_name.c_str(),
                            obs.normalized_name.c_str(),
                            obs.confidence
                        );

                        // Drop immediately when the cup scene is found, but keep scanning the rest afterward
                        if (!cupDropped && isCupMatch(obs.normalized_name, obs.confidence, 0.50f)) {
                            RCLCPP_INFO(
                                node->get_logger(),
                                "Cup detected at scene %d with confidence %.2f. Searching for AprilTag %d.",
                                obs.index, obs.confidence, obs.tag_id
                            );

                            bool dropped = localTagSearchAndDrop(
                                node, nav, tagDetector, armController, obs
                            );

                            if (dropped) {
                                cupDropped = true;
                                RCLCPP_INFO(node->get_logger(), "Cup dropped at scene %d", obs.index);
                            } else {
                                RCLCPP_WARN(node->get_logger(),
                                            "Tag-guided drop failed at scene %d. Keeping object and continuing.",
                                            obs.index);
                            }

                            bool travel_ok = moveArmToTravelPose(armController);
                            if (!travel_ok) {
                                RCLCPP_WARN(node->get_logger(), "Could not move arm to travel pose after drop/search.");
                            }
                        }
                    } else {
                        RCLCPP_WARN(node->get_logger(), "No valid contest object detected at scene %d", obs.index);
                    }
                }

                scenesVisited = true;
                state = ContestState::RETURN_TO_START;
            }
            break;
        }

        case ContestState::RETURN_TO_START:
        {
            if (!returnedToStart) {
                RCLCPP_INFO(
                    node->get_logger(),
                    "Phase 4: return to original start pose x=%.2f y=%.2f phi=%.2f",
                    start_x, start_y, start_phi
                );

                bool nav_ok = nav.moveToGoal(start_x, start_y, start_phi);
                if (!nav_ok) {
                    RCLCPP_WARN(node->get_logger(), "Failed to return to original start pose.");
                } else {
                    returnedToStart = true;
                    RCLCPP_INFO(node->get_logger(), "Robot returned to original start pose.");
                }
            }

            state = ContestState::SAVE_RESULTS;
            break;
        }

        case ContestState::SAVE_RESULTS:
        {
            if (!resultsSaved) {
                writeContestResults(
                    node,
                    manipulable_raw_name,
                    manipulable_name,
                    manipulable_confidence,
                    scene_observations
                );
                resultsSaved = true;
            }

            state = ContestState::FINISHED;
            break;
        }

        case ContestState::FINISHED:
        {
            RCLCPP_INFO(node->get_logger(), "Contest routine finished.");
            rclcpp::shutdown();
            break;
        }

        default:
            break;
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
