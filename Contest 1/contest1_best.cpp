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

inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }
inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

class Contest1Node : public rclcpp::Node
{
public:
    Contest1Node() : Node("contest1_node")
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

        start_time_ = this->now();

        angular_ = 0.0f;
        linear_  = 0.0f;

        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_   = 0.0;

        // Laser processed values
        minLaserDist_raw_ = std::numeric_limits<float>::infinity();
        left_min_raw_     = std::numeric_limits<float>::infinity();
        right_min_raw_    = std::numeric_limits<float>::infinity();

        // Filtered values (for stable control)
        center_distance_ = 12.0f;
        left_distance_   = 12.0f;
        right_distance_  = 12.0f;

        nLasers_ = 0;
        desiredAngle_ = 10; // +/- degrees around front for the min window

        state_ = 0;

        target_distance_ = 0.30;      // quicker commit
        target_rotation_ = M_PI / 2.0;

        have_start_pos_ = false;
        have_start_yaw_ = false;

        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

        grid_resolution_ = 0.25;
        last_visit_time_ = this->now();

        turn_dir_ = +1;

        stuck_timer_running_ = false;

        cooldown_active_ = false;
        cooldown_seconds_ = 0.8;

        have_scan_ = false;
        have_odom_ = false;

        // Thrash protection
        front_close_count_ = 0;
        front_blocked_latched_ = false;

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

    int chooseFrontIndex(int n, float angle_min, float angle_inc) const {
        // robust: front could be 0 or -90 depending on setup
        int mid = n / 2;
        int idx0   = angleToIndex(0.0f, angle_min, angle_inc, n);
        int idxm90 = angleToIndex((float)deg2rad(-90.0), angle_min, angle_inc, n);
        return (std::abs(idx0 - mid) < std::abs(idxm90 - mid)) ? idx0 : idxm90;
    }

    void publishCmd(float lin, float ang) {
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = lin;
        vel.twist.angular.z = ang;
        vel_pub_->publish(vel);
    }

    // simple smoothing to prevent jitter (alpha 0.25 = moderate smoothing)
    float smooth(float prev, float now, float alpha = 0.25f) const {
        return (1.0f - alpha) * prev + alpha * now;
    }

    // ---------- callbacks ----------
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        have_scan_ = true;
        if (scan->ranges.empty() || scan->angle_increment <= 0.0f) return;

        laserRange_ = scan->ranges;
        nLasers_ = (int)laserRange_.size();
        if (nLasers_ <= 0) return;

        float angle_min = (float)scan->angle_min;
        float angle_inc = (float)scan->angle_increment;

        int front_idx = chooseFrontIndex(nLasers_, angle_min, angle_inc);

        int half_window = std::max(1, (int)std::lround((float)deg2rad((double)desiredAngle_) / angle_inc));
        int start_i = clampIndex(front_idx - half_window, nLasers_);
        int end_i   = clampIndex(front_idx + half_window, nLasers_);

        // front min (raw)
        minLaserDist_raw_ = std::numeric_limits<float>::infinity();
        for (int i = start_i; i <= end_i; ++i) {
            minLaserDist_raw_ = std::min(minLaserDist_raw_, safeRange(laserRange_[i]));
        }

        // left/right mins (raw) using side windows (robust)
        left_min_raw_  = std::numeric_limits<float>::infinity();
        right_min_raw_ = std::numeric_limits<float>::infinity();

        int num_rays = half_window;
        int left_center  = clampIndex(front_idx + num_rays, nLasers_);
        int right_center = clampIndex(front_idx - num_rays, nLasers_);

        for (int i = 0; i < num_rays; ++i) {
            int li = left_center + i;
            if (li >= 0 && li < nLasers_) left_min_raw_ = std::min(left_min_raw_, safeRange(laserRange_[li]));
            int ri = right_center - i;
            if (ri >= 0 && ri < nLasers_) right_min_raw_ = std::min(right_min_raw_, safeRange(laserRange_[ri]));
        }

        // Filter (this is what fixes “stop-go-stop-go”)
        center_distance_ = smooth(center_distance_, minLaserDist_raw_);
        left_distance_   = smooth(left_distance_,   left_min_raw_);
        right_distance_  = smooth(right_distance_,  right_min_raw_);
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
        for (auto& kv : bumpers_) kv.second = false;
        for (const auto& detection : hazard_vector->detections) {
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
            }
        }
    }

    // ---------- main control ----------
    void controlLoop()
    {
        auto now = this->now();
        double seconds_elapsed = (now - start_time_).seconds();

        if (seconds_elapsed >= 480.0) {
            RCLCPP_INFO(this->get_logger(), "Contest time completed (480 seconds). Stopping robot.");
            publishCmd(0.0f, 0.0f);
            rclcpp::shutdown();
            return;
        }

        if (!have_odom_ || !have_scan_) {
            publishCmd(0.0f, 0.0f);
            return;
        }

        // Update visited every 0.5s
        if ((now - last_visit_time_).seconds() > 0.5) {
            int cx = (int)std::floor(pos_x_ / grid_resolution_);
            int cy = (int)std::floor(pos_y_ / grid_resolution_);
            visited_[{cx, cy}] = visited_[{cx, cy}] + 1;
            last_visit_time_ = now;
        }

        // cooldown
        if (cooldown_active_) {
            if ((now - cooldown_start_time_).seconds() >= cooldown_seconds_) cooldown_active_ = false;
        }

        // bumper check
        bool any_bumper = false;
        std::string pressed;
        for (const auto& kv : bumpers_) {
            if (kv.second) { any_bumper = true; pressed = kv.first; break; }
        }

        // CONSTANTS
        const float Front_Stop   = 0.50f;  // trigger turn
        const float Front_Clear  = 0.60f;  // hysteresis: must exceed this to unlatch
        const int   CloseNeeded  = 3;      // must be close for 3 consecutive ticks (0.3s at 10Hz)

        const float FastSpeed = 0.25f;     // contest max
        const float SlowSpeed = 0.10f;     // your rule near obstacles
        const float TurnSpeed = 0.75f;     // faster turns (still capped)

        const float RightWallDesired = 0.45f;
        const float WallKP = 1.2f;

        // ---- Front latch (stops the “stop-go-stop-go” jitter) ----
        if (center_distance_ < Front_Stop) front_close_count_++;
        else front_close_count_ = 0;

        if (!front_blocked_latched_ && front_close_count_ >= CloseNeeded) {
            front_blocked_latched_ = true;
        }
        // only unlatch when clearly safe (hysteresis)
        if (front_blocked_latched_ && center_distance_ > Front_Clear) {
            front_blocked_latched_ = false;
            front_close_count_ = 0;
        }

        // ---- State transitions on bumper ----
        if (any_bumper && state_ != 3 && state_ != 4) {
            turn_dir_ = +1;
            if (pressed.find("left")  != std::string::npos) turn_dir_ = -1;
            if (pressed.find("right") != std::string::npos) turn_dir_ = +1;
            if (pressed.find("front") != std::string::npos) {
                turn_dir_ = (left_distance_ >= right_distance_) ? +1 : -1;
            }
            state_ = 3;
            have_start_pos_ = false;
        }

        // -------- STATE MACHINE --------
        if (state_ == 0) {
            // If front truly blocked (latched), decide a turn
            if (front_blocked_latched_) {
                state_ = 2;
                have_start_yaw_ = false;
                turn_dir_ = (left_distance_ >= right_distance_) ? +1 : -1;
                linear_ = 0.0f;
                angular_ = 0.0f;
            } else {
                // wall-follow / exploration
                linear_ = FastSpeed;

                float wall_error = RightWallDesired - right_distance_;
                float cmd = -WallKP * wall_error;

                // If no right wall (very open), arc right to find one
                if (right_distance_ > 2.0f && center_distance_ > 1.2f) {
                    cmd = -0.55f;
                    linear_ = 0.22f;
                }

                cmd = std::max(-0.9f, std::min(cmd, 0.9f));
                angular_ = cmd;
            }
        }
        else if (state_ == 1) {
            if (!have_start_pos_) {
                start_pos_x_ = pos_x_;
                start_pos_y_ = pos_y_;
                have_start_pos_ = true;
            }
            double dist = std::hypot(pos_x_ - start_pos_x_, pos_y_ - start_pos_y_);
            if (dist < target_distance_) {
                linear_ = FastSpeed;
                angular_ = 0.0f;
            } else {
                state_ = 0;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_pos_ = false;
            }
        }
        else if (state_ == 2) {
            if (!have_start_yaw_) {
                start_yaw_ = yaw_;
                have_start_yaw_ = true;
            }

            double angle_rotated = yaw_ - start_yaw_;
            while (angle_rotated > M_PI) angle_rotated -= 2.0 * M_PI;
            while (angle_rotated < -M_PI) angle_rotated += 2.0 * M_PI;

            if (std::abs(angle_rotated) < target_rotation_) {
                linear_ = 0.0f;
                angular_ = TurnSpeed * (float)turn_dir_;
            } else {
                state_ = 0;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_yaw_ = false;
                cooldown_active_ = true;
                cooldown_start_time_ = now;
            }
        }
        else if (state_ == 3) {
            if (!have_start_pos_) {
                start_pos_x_ = pos_x_;
                start_pos_y_ = pos_y_;
                have_start_pos_ = true;
            }
            double dist = std::hypot(pos_x_ - start_pos_x_, pos_y_ - start_pos_y_);
            if (dist < 0.20) {
                linear_ = -0.12f;
                angular_ = 0.0f;
            } else {
                state_ = 4;
                have_start_yaw_ = false;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_pos_ = false;
            }
        }
        else if (state_ == 4) {
            if (!have_start_yaw_) {
                start_yaw_ = yaw_;
                have_start_yaw_ = true;
            }

            double angle_rotated = yaw_ - start_yaw_;
            while (angle_rotated > M_PI) angle_rotated -= 2.0 * M_PI;
            while (angle_rotated < -M_PI) angle_rotated += 2.0 * M_PI;

            if (std::abs(angle_rotated) < target_rotation_) {
                linear_ = 0.0f;
                angular_ = TurnSpeed * (float)turn_dir_;
            } else {
                state_ = 0;
                linear_ = 0.0f;
                angular_ = 0.0f;
                have_start_yaw_ = false;
                cooldown_active_ = true;
                cooldown_start_time_ = now;
            }
        }

        // -------- FINAL SAFETY CAPS --------
        // hard caps
        linear_  = std::max(-0.15f, std::min(linear_, 0.25f));
        angular_ = std::max(-1.05f, std::min(angular_, 1.05f));

        // YOUR RULE: slow to 0.10 within 30cm of obstacles (front/left/right)
        if (center_distance_ <= 0.30f || left_distance_ <= 0.30f || right_distance_ <= 0.30f) {
            if (linear_ > 0.10f) linear_ = 0.10f;
        }

        publishCmd(linear_, angular_);
    }

    // ROS
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // time
    rclcpp::Time start_time_;

    // command
    float angular_;
    float linear_;

    // odom
    double pos_x_;
    double pos_y_;
    double yaw_;
    bool have_odom_;

    // bumpers
    std::map<std::string, bool> bumpers_;

    // laser raw + filtered
    std::vector<float> laserRange_;
    int32_t nLasers_;
    int32_t desiredAngle_;
    bool have_scan_;

    float minLaserDist_raw_;
    float left_min_raw_;
    float right_min_raw_;

    float center_distance_;
    float left_distance_;
    float right_distance_;

    // front thrash protection
    int front_close_count_;
    bool front_blocked_latched_;

    // state machine
    int state_;
    int turn_dir_;

    double start_pos_x_ = 0.0;
    double start_pos_y_ = 0.0;
    double target_distance_;
    double start_yaw_ = 0.0;
    double target_rotation_;
    bool have_start_pos_;
    bool have_start_yaw_;

    // least-visited
    double grid_resolution_;
    std::map<std::pair<int, int>, int> visited_;
    rclcpp::Time last_visit_time_;

    // stuck (kept but minimal)
    bool stuck_timer_running_;
    double stuck_start_x_ = 0.0;
    double stuck_start_y_ = 0.0;
    rclcpp::Time stuck_start_time_;

    // cooldown
    rclcpp::Time cooldown_start_time_;
    bool cooldown_active_;
    double cooldown_seconds_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
