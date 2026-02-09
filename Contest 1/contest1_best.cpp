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

        timer_ = this->create_wall_timer(100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize variables
        start_time_ = this->now();

        angular_ = 0.0f;
        linear_ = 0.0f;

        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;

        // Laser processed values (robust)
        minLaserDist_ = std::numeric_limits<float>::infinity();
        left_min_dist_ = std::numeric_limits<float>::infinity();
        right_min_dist_ = std::numeric_limits<float>::infinity();

        // Derived "directional" distances used by your state machine
        center_distance_ = 12.0f;
        left_distance_   = 12.0f;
        right_distance_  = 12.0f;

        nLasers_ = 0;
        desiredAngle_ = 10; // +/- around front for minLaserDist_

        state_ = 0;

        start_pos_x_ = 0.0;
        start_pos_y_ = 0.0;
        target_distance_ = 0.40;

        start_yaw_ = 0.0;
        target_rotation_ = M_PI / 2.0;

        have_start_pos_ = false;
        have_start_yaw_ = false;

        // Bumper states
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

        // Least-visited memory
        grid_resolution_ = 0.25;
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
        cooldown_seconds_ = 1.0;

        // Scan parameters
        scan_angle_min_ = 0.0;
        scan_angle_inc_ = 0.0;
        have_scan_params_ = false;

        // Startup safety flags (big difference vs your “goes straight” issue)
        have_scan_ = false;
        have_odom_ = false;

        // L-pocket escape timer
        pocket_timer_running_ = false;
        pocket_start_time_ = this->now();
        pocket_hold_seconds_ = 1.5;

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    // ---------- helpers ----------
    int clampIndex(int idx, int n) const {
        if (n <= 0) return 0;
        if (idx < 0) return 0;
        if (idx >= n) return n - 1;
        return idx;
    }

    int angleToIndex(float angle_rad, float angle_min, float angle_inc, int n) const {
        if (n <= 0 || angle_inc <= 0.0f) return 0;
        int idx = (int)std::lround((angle_rad - angle_min) / angle_inc);
        return clampIndex(idx, n);
    }

    float safeRange(float r) const {
        if (!std::isfinite(r) || r <= 0.02f) return 12.0f;
        return r;
    }

    // Pick “front” robustly (some setups are front=0, some are front=-90deg)
    int chooseFrontIndex(int n, float angle_min, float angle_inc) const {
        int mid = n / 2;
        int idx0   = angleToIndex(0.0f, angle_min, angle_inc, n);
        int idxm90 = angleToIndex((float)deg2rad(-90.0), angle_min, angle_inc, n);
        return (std::abs(idx0 - mid) < std::abs(idxm90 - mid)) ? idx0 : idxm90;
    }

    // ---------- callbacks ----------
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        have_scan_ = true;

        if (scan->ranges.empty() || scan->angle_increment <= 0.0f) {
            return;
        }

        laserRange_ = scan->ranges;
        nLasers_ = (int)laserRange_.size();

        scan_angle_min_ = (float)scan->angle_min;
        scan_angle_inc_ = (float)scan->angle_increment;
        have_scan_params_ = (nLasers_ > 0);

        if (!have_scan_params_) return;

        // choose front index robustly
        int front_idx = chooseFrontIndex(nLasers_, scan_angle_min_, scan_angle_inc_);

        // (A) minLaserDist_ in +/- desiredAngle_ around front
        int half_window = (int)std::lround((float)deg2rad((double)desiredAngle_) / scan_angle_inc_);
        int start_i = clampIndex(front_idx - half_window, nLasers_);
        int end_i   = clampIndex(front_idx + half_window, nLasers_);

        minLaserDist_ = std::numeric_limits<float>::infinity();
        for (int i = start_i; i <= end_i; ++i) {
            minLaserDist_ = std::min(minLaserDist_, safeRange(laserRange_[i]));
        }

        // (B) left/right “open-space mins” like your working code (robust, not single-ray)
        left_min_dist_  = std::numeric_limits<float>::infinity();
        right_min_dist_ = std::numeric_limits<float>::infinity();

        int num_rays = std::max(1, half_window); // reuse a similar window size
        int left_center  = clampIndex(front_idx + num_rays, nLasers_);
        int right_center = clampIndex(front_idx - num_rays, nLasers_);

        for (int i = 0; i < num_rays; ++i) {
            int li = left_center + i;
            if (li >= 0 && li < nLasers_) {
                float r = safeRange(laserRange_[li]);
                left_min_dist_ = std::min(left_min_dist_, r);
            }
            int ri = right_center - i;
            if (ri >= 0 && ri < nLasers_) {
                float r = safeRange(laserRange_[ri]);
                right_min_dist_ = std::min(right_min_dist_, r);
            }
        }

        // Provide robust directional distances to your existing logic
        center_distance_ = safeRange(minLaserDist_);
        left_distance_   = safeRange(left_min_dist_);
        right_distance_  = safeRange(right_min_dist_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);
        have_odom_ = true;
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // Reset known bumpers
        for (auto& kv : bumpers_) kv.second = false;

        // Update bumper states
        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true; // ok even if a new key appears
            }
        }
    }

    void publishStopAndReturn()
    {
        linear_ = 0.0f;
        angular_ = 0.0f;
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = 0.0;
        vel.twist.angular.z = 0.0;
        vel_pub_->publish(vel);
    }

    // ---------- main control ----------
    void controlLoop()
    {
        auto current_time = this->now();
        double seconds_elapsed = (current_time - start_time_).seconds();

        if (seconds_elapsed >= 480.0) {
            RCLCPP_INFO(this->get_logger(), "Contest time completed (480 seconds). Stopping robot.");
            publishStopAndReturn();
            rclcpp::shutdown();
            return;
        }

        // Startup safety: don’t drive until both arrive
        if (!have_odom_ || !have_scan_) {
            publishStopAndReturn();
            return;
        }

        // Update visited cell counter every 0.5 sec
        if ((this->now() - last_visit_time_).seconds() > 0.5) {
            int cx = (int)std::floor(pos_x_ / grid_resolution_);
            int cy = (int)std::floor(pos_y_ / grid_resolution_);
            visited_[{cx, cy}] = visited_[{cx, cy}] + 1;
            last_visit_time_ = this->now();
        }

        // Cooldown timer update
        if (cooldown_active_) {
            double cool_dt = (this->now() - cooldown_start_time_).seconds();
            if (cool_dt >= cooldown_seconds_) cooldown_active_ = false;
        }

        // Bumper iteration
        bool any_bumper_pressed = false;
        std::string pressed_bumper = "";

        for (const auto& kv : bumpers_) {
            if (kv.second) {
                any_bumper_pressed = true;
                pressed_bumper = kv.first;
                break;
            }
        }

        // Bumper pressed -> go to backup state
        if (any_bumper_pressed) {
            if (state_ != 3 && state_ != 4) {
                // Turn away from hit side; for center choose more open side
                turn_dir_ = +1;
                if (pressed_bumper.find("left") != std::string::npos)  turn_dir_ = -1;
                if (pressed_bumper.find("right") != std::string::npos) turn_dir_ = +1;

                if (pressed_bumper.find("front_center") != std::string::npos ||
                    pressed_bumper.find("front_left") != std::string::npos ||
                    pressed_bumper.find("front_right") != std::string::npos) {
                    turn_dir_ = (left_distance_ >= right_distance_) ? +1 : -1;
                }

                state_ = 3;
                have_start_pos_ = false;
                pocket_timer_running_ = false;
            }
        }

        // Stuck detection (only in state 0 while trying to move forward)
        if (state_ == 0 && linear_ > 0.05f) {
            if (!stuck_timer_running_) {
                stuck_timer_running_ = true;
                stuck_start_time_ = this->now();
                stuck_start_x_ = pos_x_;
                stuck_start_y_ = pos_y_;
            } else {
                double dt = (this->now() - stuck_start_time_).seconds();
                double moved = std::hypot(pos_x_ - stuck_start_x_, pos_y_ - stuck_start_y_);

                if (dt >= 2.0 && moved < 0.05) {
                    state_ = 3;
                    have_start_pos_ = false;
                    turn_dir_ = (left_distance_ >= right_distance_) ? +1 : -1;
                    stuck_timer_running_ = false;
                    pocket_timer_running_ = false;
                }
                if (moved >= 0.10) stuck_timer_running_ = false;
            }
        } else {
            stuck_timer_running_ = false;
        }

        // -------- STATE MACHINE --------
        // Use robust distances:
        // center_distance_ == minLaserDist_ (front window min)
        // left_distance_ / right_distance_ == side window mins

        // State 0: Wall-follow + junction decision (least-visited)
        if (state_ == 0) {
            const float Front_Stop = 0.50f; // slightly safer than 0.45
            const float Front_Slow = 0.80f;

            const float Max_Free = 0.25f;
            const float Max_Near = 0.10f;

            const float Right_Wall_Desired = 0.45f;
            const float Wall_KP = 1.2f;

            if (center_distance_ < Front_Stop) {
                state_ = 2;
                have_start_yaw_ = false;

                turn_dir_ = (left_distance_ >= right_distance_) ? +1 : -1;
                target_rotation_ = M_PI / 2.0;

                linear_ = 0.0f;
                angular_ = 0.0f;
                pocket_timer_running_ = false;
            } else {
                bool straight_open = (center_distance_ > 1.20f);
                bool left_open     = (left_distance_   > 1.00f);
                bool right_open    = (right_distance_  > 1.00f);

                // Least-visited junction decision
                if (!cooldown_active_ && straight_open && (left_open || right_open)) {
                    double look = 0.60;

                    int s_count = 999999, l_count = 999999, r_count = 999999;

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
                    } else if (r_count <= l_count) {
                        state_ = 2;
                        have_start_yaw_ = false;
                        turn_dir_ = -1;
                        target_rotation_ = M_PI / 2.0;
                        linear_ = 0.0f;
                        angular_ = 0.0f;
                    } else {
                        state_ = 2;
                        have_start_yaw_ = false;
                        turn_dir_ = +1;
                        target_rotation_ = M_PI / 2.0;
                        linear_ = 0.0f;
                        angular_ = 0.0f;
                    }

                    pocket_timer_running_ = false;
                } else {
                    // Wall-follow
                    float speed_cap = Max_Free;
                    if (center_distance_ < Front_Slow) speed_cap = Max_Near;
                    if (right_distance_ < 0.35f) speed_cap = Max_Near;
                    if (left_distance_  < 0.35f) speed_cap = Max_Near;
                    linear_ = speed_cap;

                    // If right wall far away, steer right to reacquire
                    // (this prevents “go straight forever” in open spaces)
                    float wall_error = Right_Wall_Desired - right_distance_;
                    float cmd = -Wall_KP * wall_error;

                    // extra “find a right wall” bias when right is very open
                    if (right_distance_ > 2.0f && center_distance_ > 1.2f) {
                        cmd = -0.45f;
                        linear_ = 0.18f;
                    }

                    cmd = std::max(-0.8f, std::min(cmd, 0.8f));
                    angular_ = cmd;

                    // L-pocket escape (right very open, left close, front open for a while) -> force one forward commit
                    bool pocket_now = (!cooldown_active_ && right_distance_ > 1.8f && left_distance_ < 0.7f && center_distance_ > 0.9f);
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
                                pocket_timer_running_ = false;
                            }
                        }
                    } else {
                        pocket_timer_running_ = false;
                    }
                }
            }
        }

        // State 1: Move fixed distance using odom
        else if (state_ == 1) {
            pocket_timer_running_ = false;

            if (!have_start_pos_) {
                start_pos_x_ = pos_x_;
                start_pos_y_ = pos_y_;
                have_start_pos_ = true;
            }

            double dist = std::hypot(pos_x_ - start_pos_x_, pos_y_ - start_pos_y_);
            if (dist < target_distance_) {
                linear_ = 0.25f;
                angular_ = 0.0f;
            } else {
                state_ = 0;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_pos_ = false;
            }
        }

        // State 2: Turn fixed angle using yaw
        else if (state_ == 2) {
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
            } else {
                state_ = 0;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_yaw_ = false;

                cooldown_active_ = true;
                cooldown_start_time_ = this->now();
            }
        }

        // State 3: Backup using odom
        else if (state_ == 3) {
            pocket_timer_running_ = false;

            if (!have_start_pos_) {
                start_pos_x_ = pos_x_;
                start_pos_y_ = pos_y_;
                have_start_pos_ = true;
            }

            double dist = std::hypot(pos_x_ - start_pos_x_, pos_y_ - start_pos_y_);
            if (dist < 0.20) {
                linear_ = -0.10f;
                angular_ = 0.0f;
            } else {
                state_ = 4;
                have_start_yaw_ = false;
                target_rotation_ = M_PI / 2.0;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_pos_ = false;
            }
        }

        // State 4: Turn after backup using yaw
        else if (state_ == 4) {
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
            } else {
                state_ = 0;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_yaw_ = false;

                cooldown_active_ = true;
                cooldown_start_time_ = this->now();
            }
        }

        // -------- final speed caps (contest safe) --------
        if (linear_ > 0.25f) linear_ = 0.25f;
        if (linear_ < -0.15f) linear_ = -0.15f;

        if (angular_ > (float)(M_PI / 3.0)) angular_ = (float)(M_PI / 3.0);
        if (angular_ < (float)(-M_PI / 3.0)) angular_ = (float)(-M_PI / 3.0);

        // Extra slow-down if any direction is near
        if (center_distance_ <= 0.30f || left_distance_ <= 0.30f || right_distance_ <= 0.30f) {
            if (linear_ > 0.10f) linear_ = 0.10f;
        }

        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;
        vel_pub_->publish(vel);
    }

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Time
    rclcpp::Time start_time_;

    // Cmd
    float angular_;
    float linear_;

    // Odom
    double pos_x_;
    double pos_y_;
    double yaw_;
    bool have_odom_;

    // Bumpers
    std::map<std::string, bool> bumpers_;

    // Laser
    int32_t nLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;
    bool have_scan_;
    bool have_scan_params_;
    float scan_angle_min_;
    float scan_angle_inc_;

    // Robust min distances
    float minLaserDist_;
    float left_min_dist_;
    float right_min_dist_;

    // Distances used by logic (derived robustly)
    float center_distance_;
    float left_distance_;
    float right_distance_;

    // State machine
    int state_;

    // Move/turn targets
    double start_pos_x_;
    double start_pos_y_;
    double target_distance_;
    double start_yaw_;
    double target_rotation_;
    bool have_start_pos_;
    bool have_start_yaw_;
    int turn_dir_;

    // Least-visited memory
    double grid_resolution_;
    std::map<std::pair<int, int>, int> visited_;
    rclcpp::Time last_visit_time_;

    // Stuck detection
    double stuck_start_x_;
    double stuck_start_y_;
    rclcpp::Time stuck_start_time_;
    bool stuck_timer_running_;

    // Cooldown
    rclcpp::Time cooldown_start_time_;
    bool cooldown_active_;
    double cooldown_seconds_;

    // Pocket escape
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
