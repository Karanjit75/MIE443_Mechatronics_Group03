#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "irobot_create_msgs/msg/hazard_detection_vector.hpp"
#include "tf2/utils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

// Utility functions for angle conversion
inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node()
        : Node("contest1_node")
    {
        // Initialize publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));

        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize variables
        start_time_ = this->now();

        angular_ = 0.0;
        linear_ = 0.0;

        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;

        minLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;

        state_ = 0;

        start_pos_x_ = 0.0;
        start_pos_y_ = 0.0;
        target_distance_ = 0.40; // commit forward distance at junctions (tune if needed)

        start_yaw_ = 0.0;
        target_rotation_ = M_PI / 2.0;  // 90deg (used when turning left or right)

        have_start_pos_ = false;
        have_start_yaw_ = false;

        // Bumper states
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

        // Least-visited memory
        grid_resolution_ = 0.25;    // meters per cell
        last_visit_time_ = this->now();

        // Turn directions (+1=left/CCW, -1=right/CW)
        turn_dir_ = +1;

        // Stuck detection
        stuck_start_x_ = 0.0;
        stuck_start_y_ = 0.0;
        stuck_start_time_ = this->now();
        stuck_timer_running_ = false;

        // Cooldown turns
        cooldown_start_time_ = this->now();
        cooldown_active_ = false;
        cooldown_seconds_ = 1.0; // 1 second cooldown

        // store scan parameters so controlLoop indexing matches the -90deg convention used in laserCallback
        scan_angle_min_ = 0.0;
        scan_angle_inc_ = 0.0;
        have_scan_params_ = false;

        // L-pocket escape timer (prevents spiraling into right-side alcoves)
        pocket_timer_running_ = false;
        pocket_start_time_ = this->now();
        pocket_hold_seconds_ = 1.5; // must look like a pocket for this long

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        nLasers_ = (scan->angle_max - scan->angle_min) / scan->angle_increment;
        laserRange_ = scan->ranges;
        desiredNLasers_ = deg2rad(desiredAngle_) / scan->angle_increment;

        // save scan parameters so controlLoop can compute consistent indices
        scan_angle_min_ = scan->angle_min;
        scan_angle_inc_ = scan->angle_increment;
        have_scan_params_ = true;

        // LiDAR has 90-degree offset, so front of robot is at -90 degrees in scan frame
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;

        minLaserDist_ = std::numeric_limits<float>::infinity();

        // Find minimum laser distance within +/- desiredAngle from front center
        if (deg2rad(desiredAngle_) < scan->angle_max && deg2rad(desiredAngle_) > scan->angle_min) {

            int start_i = (int)front_idx - (int)desiredNLasers_;
            int end_i = (int)front_idx + (int)desiredNLasers_;
            if (start_i < 0) start_i = 0;
            if (!laserRange_.empty() && end_i >= (int)laserRange_.size()) end_i = (int)laserRange_.size() - 1;

            for (int laser_idx = start_i; laser_idx <= end_i; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < (uint32_t)nLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // Extract position
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        // Extract yaw from quaternion using tf2
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        //RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // Reset all bumpers to released state
        for (auto& [key, val] : bumpers_) {
            val = false;
        }

        // Update bumper states based on current detections
        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                RCLCPP_INFO(this->get_logger(), "Bumper pressed: %s", detection.header.frame_id.c_str());
            }
        }
    }

    void controlLoop()
    {
        // Calculate elapsed time
        auto current_time = this->now();
        double seconds_elapsed = (current_time - start_time_).seconds();

        // Check if 480 seconds (8 minutes) have elapsed
        if (seconds_elapsed >= 480.0) {
            RCLCPP_INFO(this->get_logger(), "Contest time completed (480 seconds). Stopping robot.");

            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;
            vel_pub_->publish(vel);

            rclcpp::shutdown();
            return;
        }
        else {
            // Update visited cell counter every 0.5sec (least-visited)
            if ((this->now() - last_visit_time_).seconds() > 0.5) {
                int cx = (int)std::floor(pos_x_ / grid_resolution_);
                int cy = (int)std::floor(pos_y_ / grid_resolution_);
                visited_[{cx, cy}] = visited_[{cx, cy}] + 1;
                last_visit_time_ = this->now();
            }

            // LiDAR Indexing (center/left/right)
            float center_distance_ = 12.0f;
            float left_distance_ = 12.0f;
            float right_distance_ = 12.0f;
            if (nLasers_ > 0 && (int)laserRange_.size() == nLasers_ && have_scan_params_) {
                // Compute indices using same -90deg "front" convention as laserCallback
                float front_angle = (float)deg2rad(-90.0);
                int front_index = (int)((front_angle - (float)scan_angle_min_) / (float)scan_angle_inc_);
                int left_index = (int)(((front_angle + (float)deg2rad(90.0)) - (float)scan_angle_min_) / (float)scan_angle_inc_);
                int right_index = (int)(((front_angle - (float)deg2rad(90.0)) - (float)scan_angle_min_) / (float)scan_angle_inc_);

                if (front_index < 0) front_index = 0;
                if (left_index < 0) left_index = 0;
                if (right_index < 0) right_index = 0;
                if (front_index >= (int)laserRange_.size()) front_index = (int)laserRange_.size() - 1;
                if (left_index >= (int)laserRange_.size()) left_index = (int)laserRange_.size() - 1;
                if (right_index >= (int)laserRange_.size()) right_index = (int)laserRange_.size() - 1;

                center_distance_ = laserRange_[front_index];
                left_distance_ = laserRange_[left_index];
                right_distance_ = laserRange_[right_index];

                // Account for invalid distances
                if (!std::isfinite(center_distance_) || center_distance_ <= 0.02f) center_distance_ = 12.0f;
                if (!std::isfinite(left_distance_) || left_distance_ <= 0.02f) left_distance_ = 12.0f;
                if (!std::isfinite(right_distance_) || right_distance_ <= 0.02f) right_distance_ = 12.0f;
            }

            // Cooldown timer update
            if (cooldown_active_) {
                double cool_dt = (this->now() - cooldown_start_time_).seconds();
                if (cool_dt >= cooldown_seconds_) {
                    cooldown_active_ = false;
                }
            }

            // Bumper iteration
            bool any_bumper_pressed = false;
            std::string pressed_bumper = "";

            for (const auto& [key, value] : bumpers_) {
                if (value) {
                    any_bumper_pressed = true;
                    pressed_bumper = key;
                    RCLCPP_INFO(this->get_logger(), "Bumper %s is pressed.", key.c_str());
                    break;
                }
            }

            // If bumper pressed -> go to backup state (State 3)
            if (any_bumper_pressed) {
                if (state_ != 3 && state_ != 4) {
                    turn_dir_ = +1;

                    if (pressed_bumper.find("left") != std::string::npos) {
                        turn_dir_ = -1;
                    }
                    if (pressed_bumper.find("right") != std::string::npos) {
                        turn_dir_ = +1;
                    }

                    // Handle front_center by turning towards more open side
                    if (pressed_bumper.find("front_center") != std::string::npos) {
                        if (left_distance_ >= right_distance_) {
                            turn_dir_ = +1;
                        } else {
                            turn_dir_ = -1;
                        }
                    }

                    state_ = 3; // backup
                    have_start_pos_ = false;

                    // reset pocket timer on recovery so it doesn't trigger immediately after
                    pocket_timer_running_ = false;
                }
            }

            // Stuck detection (odom)
            if (state_ == 0 && linear_ > 0.05f) {
                if (!stuck_timer_running_) {
                    stuck_timer_running_ = true;
                    stuck_start_time_ = this->now();
                    stuck_start_x_ = pos_x_;
                    stuck_start_y_ = pos_y_;
                }
                else {
                    double dt = (this->now() - stuck_start_time_).seconds();
                    double moved = std::sqrt(std::pow(pos_x_ - stuck_start_x_, 2) + std::pow(pos_y_ - stuck_start_y_, 2));

                    if (dt >= 2.0 && moved < 0.05) {
                        RCLCPP_INFO(this->get_logger(), "Stuck detected, robot recovering now.");

                        state_ = 3;
                        have_start_pos_ = false;

                        if (left_distance_ >= right_distance_) {
                            turn_dir_ = +1;
                        }
                        else {
                            turn_dir_ = -1;
                        }
                        stuck_timer_running_ = false;

                        // CHANGE: reset pocket timer on recovery
                        pocket_timer_running_ = false;
                    }
                    if (moved >= 0.10) {
                        stuck_timer_running_ = false;
                    }
                }
            }
            else {
                stuck_timer_running_ = false;
            }

            // State 0: Wall-Follow + Junction decision (least-visited)
            if (state_ == 0) {
                float Front_Stop = 0.45f;
                float Front_Slow = 0.80f;

                float Max_Free = 0.25f;
                float Max_Near = 0.10f;

                float Right_Wall_Desired = 0.45f;
                float Wall_KP = 1.2f;

                if (center_distance_ < Front_Stop) {
                    state_ = 2;
                    have_start_yaw_ = false;

                    if (left_distance_ >= right_distance_) {
                        turn_dir_ = +1;
                    }
                    else {
                        turn_dir_ = -1;
                    }

                    target_rotation_ = M_PI / 2.0;

                    linear_ = 0.0f;
                    angular_ = 0.0f;

                    // CHANGE: reset pocket timer when switching out of wall-follow
                    pocket_timer_running_ = false;
                }
                else {
                    bool straight_open = false;
                    bool left_open = false;
                    bool right_open = false;

                    if (center_distance_ > 1.20f) straight_open = true;
                    if (left_distance_ > 1.00f) left_open = true;
                    if (right_distance_ > 1.00f) right_open = true;

                    if (!cooldown_active_ && straight_open && (left_open || right_open)) {
                        double look = 0.60;

                        int s_count = 999999;
                        int l_count = 999999;
                        int r_count = 999999;

                        if (straight_open) {
                            double sx = pos_x_ + look * std::cos(yaw_);
                            double sy = pos_y_ + look * std::sin(yaw_);
                            int scx = (int)std::floor(sx / grid_resolution_);
                            int scy = (int)std::floor(sy / grid_resolution_);
                            s_count = visited_[{scx, scy}];
                        }

                        if (left_open) {
                            double lx = pos_x_ + look * std::cos(yaw_ + M_PI/2.0);
                            double ly = pos_y_ + look * std::sin(yaw_ + M_PI/2.0);
                            int lcx = (int)std::floor(lx / grid_resolution_);
                            int lcy = (int)std::floor(ly / grid_resolution_);
                            l_count = visited_[{lcx, lcy}];
                        }

                        if (right_open) {
                            double rx = pos_x_ + look * std::cos(yaw_ - M_PI/2.0);
                            double ry = pos_y_ + look * std::sin(yaw_ - M_PI/2.0);
                            int rcx = (int)std::floor(rx / grid_resolution_);
                            int rcy = (int)std::floor(ry / grid_resolution_);
                            r_count = visited_[{rcx, rcy}];
                        }

                        if (s_count <= r_count && s_count <= l_count) {
                            state_ = 1;
                            target_distance_ = 0.40;
                            have_start_pos_ = false;
                            linear_ = 0.0f;
                            angular_ = 0.0f;
                        }
                        else if (r_count <= l_count) {
                            state_ = 2;
                            have_start_yaw_ = false;
                            turn_dir_ = -1;
                            target_rotation_ = M_PI / 2.0;
                            linear_ = 0.0f;
                            angular_ = 0.0f;
                        }
                        else {
                            state_ = 2;
                            have_start_yaw_ = false;
                            turn_dir_ = +1;
                            target_rotation_ = M_PI / 2.0;
                            linear_ = 0.0f;
                            angular_ = 0.0f;
                        }

                        // reset pocket timer when leaving wall-follow path
                        pocket_timer_running_ = false;
                    }
                    else {
                        float speed_cap = Max_Free;
                        if (center_distance_ < Front_Slow) speed_cap = Max_Near;
                        if (right_distance_ < 0.35f) speed_cap = Max_Near;
                        if (left_distance_ < 0.35f) speed_cap = Max_Near;
                        linear_ = speed_cap;

                        float wall_error = Right_Wall_Desired - right_distance_;
                        float cmd = -Wall_KP * wall_error;

                        if (cmd > 0.8f) cmd = 0.8f;
                        if (cmd < -0.8f) cmd = -0.8f;
                        angular_ = cmd;

                        // L-pocket escape (right very open, left close, front open for a while) -> force one forward commit
                        bool pocket_now = false;
                        if (!cooldown_active_ && right_distance_ > 1.8f && left_distance_ < 0.7f && center_distance_ > 0.9f) {
                            pocket_now = true;
                        }

                        if (pocket_now) {
                            if (!pocket_timer_running_) {
                                pocket_timer_running_ = true;
                                pocket_start_time_ = this->now();
                            } else {
                                double pocket_dt = (this->now() - pocket_start_time_).seconds();
                                if (pocket_dt >= pocket_hold_seconds_) {
                                    state_ = 1;
                                    target_distance_ = 0.40;
                                    have_start_pos_ = false;
                                    linear_ = 0.0f;
                                    angular_ = 0.0f;

                                    pocket_timer_running_ = false; // one-shot
                                }
                            }
                        } else {
                            pocket_timer_running_ = false;
                        }

                        // original behavior kept (only runs if we didn't switch state above)
                        if (state_ == 0) {
                            if (right_distance_ > 1.5f && center_distance_ > 1.0f) {
                                angular_ = -0.35f;
                            }
                        }
                    }
                }
            }

            // State 1: Move fixed distance using odom
            else if (state_ == 1) {
                // reset pocket timer when in commit-forward state
                pocket_timer_running_ = false;

                if (!have_start_pos_) {
                    start_pos_x_ = pos_x_;
                    start_pos_y_ = pos_y_;
                    have_start_pos_ = true;
                }

                double distance_traveled = std::sqrt(std::pow(pos_x_ - start_pos_x_, 2) + std::pow(pos_y_ - start_pos_y_, 2));

                if (distance_traveled < target_distance_) {
                    linear_ = 0.25f;
                    angular_ = 0.0f;
                    RCLCPP_INFO(this->get_logger(), "Moving forward: %.3f / %.3f m", distance_traveled, target_distance_);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "Reached target distance, returning to wall follow.");
                    state_ = 0;
                    linear_ = 0.0f;
                    angular_ = 0.0f;
                    have_start_pos_ = false;
                }
            }

            // State 2: Turn fixed angle using Yaw
            else if (state_ == 2) {
                // reset pocket timer when turning
                pocket_timer_running_ = false;

                if (!have_start_yaw_) {
                    start_yaw_ = yaw_;
                    have_start_yaw_ = true;
                }

                double angle_rotated = yaw_ - start_yaw_;
                while (angle_rotated > M_PI) angle_rotated -= 2.0 * M_PI;
                while (angle_rotated < -M_PI) angle_rotated += 2.0 * M_PI;

                if (std::abs(angle_rotated) < target_rotation_) {
                    linear_ = 0.0f;
                    angular_ = 0.5f * (float)turn_dir_;
                    RCLCPP_INFO(this->get_logger(), "Rotating: %.1f / %.1f deg", rad2deg(std::abs(angle_rotated)), rad2deg(target_rotation_));
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "Reached target rotation, returning to wall follow.");
                    state_ = 0;
                    linear_ = 0.0f;
                    angular_ = 0.0f;
                    have_start_yaw_ = false;

                    cooldown_active_ = true;
                    cooldown_start_time_ = this->now();
                }
            }

            // State 3: Bumper Backup using Odom
            else if (state_ == 3) {
                // reset pocket timer when backing up
                pocket_timer_running_ = false;

                if (!have_start_pos_) {
                    start_pos_x_ = pos_x_;
                    start_pos_y_ = pos_y_;
                    have_start_pos_ = true;
                }

                double distance_traveled = std::sqrt(std::pow(pos_x_ - start_pos_x_, 2) + std::pow(pos_y_ - start_pos_y_, 2));

                if (distance_traveled < 0.20) {
                    linear_ = -0.10f;
                    angular_ = 0.0f;
                }
                else {
                    state_ = 4;
                    have_start_yaw_ = false;
                    target_rotation_ = M_PI / 2.0;
                    linear_ = 0.0f;
                    angular_ = 0.0f;
                    have_start_pos_ = false;
                }
            }

            // State 4: Bumper Turn using Yaw
            else if (state_ == 4) {
                // reset pocket timer when turning
                pocket_timer_running_ = false;

                if (!have_start_yaw_) {
                    start_yaw_ = yaw_;
                    have_start_yaw_ = true;
                }

                double angle_rotated = yaw_ - start_yaw_;
                while (angle_rotated > M_PI) angle_rotated -= 2.0 * M_PI;
                while (angle_rotated < -M_PI) angle_rotated += 2.0 * M_PI;

                if (std::abs(angle_rotated) < target_rotation_) {
                    linear_ = 0.0f;
                    angular_ = 0.5f * (float)turn_dir_;
                }
                else {
                    state_ = 0;
                    linear_ = 0.0f;
                    angular_ = 0.0f;
                    have_start_yaw_ = false;

                    cooldown_active_ = true;
                    cooldown_start_time_ = this->now();
                }
            }

            // Final safety speed caps
            if (linear_ > 0.25f) linear_ = 0.25f;
            if (linear_ < -0.15f) linear_ = -0.15f;
            if (angular_ > (M_PI / 3)) angular_ = M_PI / 3;
            if (angular_ < (-M_PI / 3)) angular_ = -M_PI / 3;
        }

        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;
        vel_pub_->publish(vel);
    }

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Time start_time_;

    float angular_;
    float linear_;

    double pos_x_;
    double pos_y_;
    double yaw_;

    std::map<std::string, bool> bumpers_;

    float minLaserDist_;
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;

    int state_;

    double start_pos_x_;
    double start_pos_y_;
    double target_distance_;

    double start_yaw_;
    double target_rotation_;

    bool have_start_pos_;
    bool have_start_yaw_;

    int turn_dir_;

    double grid_resolution_;
    std::map<std::pair<int, int>, int> visited_;
    rclcpp::Time last_visit_time_;

    double stuck_start_x_;
    double stuck_start_y_;
    rclcpp::Time stuck_start_time_;
    bool stuck_timer_running_;

    rclcpp::Time cooldown_start_time_;
    bool cooldown_active_;
    double cooldown_seconds_;

    // Scan parameters for consistent indexing
    double scan_angle_min_;
    double scan_angle_inc_;
    bool have_scan_params_;

    // L-pocket escape timer variables
    bool pocket_timer_running_;
    rclcpp::Time pocket_start_time_;
    double pocket_hold_seconds_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
