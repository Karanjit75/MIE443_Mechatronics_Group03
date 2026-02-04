#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>       // included for use of std::numeric_limits<float>::infinity();
#include <string>       // included for use of std::map<std::string, bool> bumpers_;

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

        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

        minLaserDist_ = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 15;   // +/- 15-degree around forward (total 30-degree)

        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    static double fix(double speed, double low_speed, double high_speed) // to fix speed caps (helper function created)
    {
        return std::max(low_speed, std::min(speed, high_speed));
    }

    float choose_open_space(float left_d, float right_d) {
        // If both invalid (infinity/undefined) -> don't flip randomly, just give last direction
        if (!std::isfinite(left_d) && !std::isfinite(right_d)) {
            return flip_direction;    // stays +1 or -1 from last time
        }
        // If left side is more open:
        if (left_d > right_d + 0.001) {
            return 1.0;
        }
        // If right side is more open:
        if (right_d > left_d + 0.001) {
            return -1.0;
        }
        // If almost equal, choose alternate direction:
        flip_direction *= -1.0;
        return flip_direction;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        have_scan_ = true; // to prevent driving forward before scan arrives

        // Number of rays
        nLasers_ = ((scan->angle_max - scan->angle_min) / scan->angle_increment);
        laserRange_ = scan->ranges;
        desiredNLasers_ = deg2rad(desiredAngle_) / scan->angle_increment;
        // RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset: %d", nLasers_, desiredNLasers_);

        // LiDAR has 90-degree offset, so front of robot is at -90 degrees in scan frame
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;

        minLaserDist_ = std::numeric_limits<float>::infinity();
        left_min_dist_ = std::numeric_limits<float>::infinity(); // Check for open space on left side
        right_min_dist_ = std::numeric_limits<float>::infinity(); // Check for open space on right side

        const uint32_t num_rays = static_cast<uint32_t>(desiredNLasers_);  // Number of laser rays to look on each side
        
        const uint32_t left_center = front_idx + num_rays; // Center index for left side
        
        uint32_t right_center; // Center index for right side
        if (front_idx > num_rays) {
            // To not go negative when shifting right
            right_center = front_idx - num_rays;
        }
        else {
            right_center = 0;   // Fix 0 to avoid negatives
        }

        // Loop through rays on both left/right sides
        for (uint32_t i = 0; i < num_rays; ++i) {
            // Left side
            uint32_t left_idx = left_center + i; // Actual index on left side for laser ray
            // to check if valid value (not infinity or undefined value)
            if (left_idx < laserRange_.size() && std::isfinite(laserRange_[left_idx])) { 
                left_min_dist_ = std::min(left_min_dist_, laserRange_[left_idx]); // store minimum distance on left side
            }
            
            // Right side
            if (right_center >= i) {
                uint32_t right_idx = right_center - i;
                // to check if valid value (not infinity or undefined value)
                if (right_idx < laserRange_.size() && std::isfinite(laserRange_[right_idx])) {
                    right_min_dist_ = std::min(right_min_dist_, laserRange_[right_idx]); // store minimum distance on right side
                }
            }
        }

        // Find minimum laser distance within +/- desiredAngle from front center
        if (deg2rad(desiredAngle_) < scan->angle_max && deg2rad(desiredAngle_) > scan->angle_min) {
            // start_idx and end_idx added to avoid underflow or overflow (safety)
            uint32_t start_idx;
            if (front_idx > static_cast<uint32_t>(desiredNLasers_)) {
                start_idx = front_idx - desiredNLasers_;
            }
            else {
                start_idx = 0;
            }

            uint32_t end_idx = front_idx + desiredNLasers_;
            if (end_idx >= laserRange_.size()) {
                end_idx = static_cast<uint32_t>(laserRange_.size() - 1);
            }
            
            minLaserDist_ = std::numeric_limits<float>::infinity();
            for (uint32_t laser_idx = start_idx; laser_idx <= end_idx; ++laser_idx) {
                if (std::isfinite(laserRange_[laser_idx])) {
                    minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
                }
            }
        }
        else {
            // Scan everyhting safely
            minLaserDist_ = std::numeric_limits<float>::infinity();
            for (uint32_t laser_idx = 0; laser_idx < static_cast<uint32_t>(nLasers_) && laser_idx < laserRange_.size(); ++laser_idx) {
                if (std::isfinite(laserRange_[laser_idx])) {
                    minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
                }
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

        have_odom_ = true;
        // RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));

        // To initialize stuck detection on first odom (starting point to compare movement)
        if (!init_progress_) {
            last_check_time = this->now();
            last_check_x_ = pos_x_;
            last_check_y_ = pos_y_;
            init_progress_ = true;
        }
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // If any hazard is detected, flag it

        // Reset all bumpers to released state
        for (auto& [key, val] : bumpers_) {
            val = false;
        }

        // Update bumper states based on current detections
        for (const auto& detection : hazard_vector->detections) {
            // HazardDetection type: only physical bumper contact
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

            // Stop the robot
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = 0.0;
            vel.twist.angular.z = 0.0;
            vel_pub_->publish(vel);

            // Shutdown the node
            rclcpp::shutdown();
            return;
        }

        rclcpp::Time now = this->now();
        static bool force_recovery = false;

        // (1) Startup Robot Safety (wait for atleast 1 odom and scan to arrive -> prevents blindly driving forward during start up)
        if (!have_odom_) {
            linear_ = 0.0;
            angular_ = 0.0;
            // Set velocity command
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = linear_;
            vel.twist.angular.z = angular_;
            // Publish velocity command
            vel_pub_->publish(vel);
            return;
        }
 
        if (!have_scan_) {
            linear_ = 0.0;
            angular_ = 0.0;
            // Set velocity command
            geometry_msgs::msg::TwistStamped vel;
            vel.header.stamp = this->now();
            vel.twist.linear.x = linear_;
            vel.twist.angular.z = angular_;
            // Publish velocity command
            vel_pub_->publish(vel);
            return;
        }        

        // (2) Constants Fixed for Control loop (contest-safe)
        const float Stop_Dist = 0.50;  // hard avoidance limit
        const float Slow_Dist = 0.75;  // slow zone limit
        const float Fast_Speed = 0.25; // speed cap <=0.25 m/s (contest maximum)
        const float Slow_Speed = 0.10; // speed cap <= 0.10 m/s (near obstacles)
        const float Turn_Speed = M_PI / 4; // Turning speed (in rad/s)

        // (3) Bumpers 
            // (3.1) Bumper States
        bool LeftPressed = bumpers_["bump_left"];
        bool RightPressed = bumpers_["bump_right"];
        bool CenterPressed = bumpers_["bump_front_center"] || bumpers_["bump_front_left"] || bumpers_["bump_front_right"];
        bool any_bumper_pressed = LeftPressed || RightPressed || CenterPressed;

            // (3.2) Latch bumper for 0.40 sec after any hit (helps most with stacked-tiles obstacle/objects under view of LiDAR)
        static rclcpp::Time last_bumped_time = now;
        if (any_bumper_pressed) {
            last_bumped_time = now;
        }
        bool bumper_latched = (now - last_bumped_time).seconds() < 0.40;
        bool bumper_active = any_bumper_pressed || bumper_latched;  // treat bumper as active if in latching state

        // (4) State memory across loops (Recovery + Turn-Lock)
        static bool recovering = false;
        static rclcpp::Time recoverStart;
        static float recoveryTurnDirection = 1.0; // +1.0 left turn, -1.0 right turn

        static bool turn_lock = false;
        static rclcpp::Time turn_lock_start;
        static float turn_lock_dir = 1.0; // +1.0 left turn, -1.0 right turn

        // (5) Stuck Detection:
            // If robot is commanded to go forward but is not moving (based on odom callback)
            // This forces recovery to escape trapped/enclosed areas or C-shaped/L-shaped obstacles
            const double check_period = 2.0; // time (in sec) between odom checks
            const double min_progress = 0.05; // 5cm
            const double want_forward = 0.05; // threshold for 'we intended to move forward'

            if (init_progress_) {
                double stuck_check_time = (now - last_check_time).seconds();

                if (stuck_check_time >= check_period) {
                    double moved_dist = std::hypot(pos_x_ - last_check_x_, pos_y_ - last_check_y_); // distance moved is sqrt(dx^2 + dy^2) - diagonal between them

                    // Update for next movement comparison
                    last_check_time = now;
                    last_check_x_ = pos_x_;
                    last_check_y_ = pos_y_;

                    bool near_obstacle = (minLaserDist_ < Slow_Dist);
                    if (!recovering && linear_ > want_forward && moved_dist < min_progress && near_obstacle) {
                        RCLCPP_WARN(this->get_logger(), "Stuck detected: moved %.3f m in %.1f sec. Forcing recovery state.", moved_dist, check_period);
                        force_recovery = true;
                    }
                }
            }
        
        // (6) Recovery mode (if bumper hit or forced recovery requested for stuck detection)
            // side bumper (left or right) -> turn away from hit side
            // center/forced -> turn towards more open side (using left/right minimum distances)
        if ((bumper_active || force_recovery) && !recovering) {
            recovering = true;
            force_recovery = false;
            recoverStart = now;

            // Choose turn direction based on which bumper was hit
            if (LeftPressed) {
                recoveryTurnDirection = -1.0; // Left bumper hit -> turn right
            }
            else if (RightPressed) {
                recoveryTurnDirection = 1.0; // Right bumper hit -> turn left 
            }
            else {
                // Center bumper hit or forced recovery -> choose more open space (call helper function)
                recoveryTurnDirection = choose_open_space(left_min_dist_, right_min_dist_);
            }
            turn_lock = false;                   
            RCLCPP_WARN(this->get_logger(), "Recovery started: Turn direction = %.0f", recoveryTurnDirection);
        }

        // (7) Different Modes (Recovery Mode, Normal Mode)
        linear_ = 0.0;
        angular_ = 0.0;

        if (recovering) {
            // Recovery: reverse -> turn -> forward arc motion
            double recovery_time = (now - recoverStart).seconds();

            if (recovery_time < 0.90) {
                // Reverse to clear obstacle footprint
                linear_ = -0.1;
                angular_ = 0.0;
            }
            else if (recovery_time < 2.10) {
                // Turn away to change heading direction
                linear_ = 0.0;
                angular_ = Turn_Speed * recoveryTurnDirection;
            }
            else if (recovery_time < 2.90) {
                // Move in a forward arc to avoid re-contact with the obstacle
                linear_ = 0.08;
                angular_ = 0.55 * recoveryTurnDirection;
            }
            else {
                // Recovery done
                recovering = false;
                linear_ = 0.0;
                angular_ = 0.0;
            }
        }
        else {
            // Normal: forward drive + corner handling
            bool too_close = (minLaserDist_ < Stop_Dist);
            if (too_close) {
                // Lock a direction for short time to stop robot from jittering left/right
                if (!turn_lock) {
                    turn_lock = true;
                    turn_lock_start = now;
                    turn_lock_dir = choose_open_space(left_min_dist_, right_min_dist_);
                }
                // Unlock after a short time (0.9 sec) and 5cm (for front clearance)
                double t_lock = (now - turn_lock_start).seconds();
                if (t_lock > 0.90 && minLaserDist_ > Stop_Dist + 0.05) {
                    turn_lock = false;
                }
                // Reverse arc (better than spinning in-place in corners)
                linear_ = -0.05;
                angular_ = Turn_Speed * turn_lock_dir;     
            }
            else {
                // If safe, then drive forward. Slow down near obstacles.
                if (minLaserDist_ < Slow_Dist) {
                    linear_ = Slow_Speed;
                }
                else {
                    linear_ = Fast_Speed;
                }
                // Gently steer away from closer side to reduce corner grazing
                float steer_dir = 0.0;
                if (left_min_dist_ < right_min_dist_ - 0.001) {
                    steer_dir = -1.0; // left closer -> steer right
                }
                if (right_min_dist_ < left_min_dist_ - 0.001) {
                    steer_dir = 1.0; // right closer -> steer left
                }
                angular_ = 0.35 * steer_dir;
            }
        }   

        // (8) Speed caps (extra safety layer added to meet contest requirements)
            // Hard speed cap:
        linear_ = static_cast<float>(fix(linear_, -0.25, 0.25));

            // Slow zone (obstacles) speed cap:
        if (minLaserDist_ < Slow_Dist) {
            linear_ = static_cast<float>(fix(linear_, -0.10, 0.10));
        }

            // Angular speed cap (to prevent violent spinning of the robot):
        angular_ = static_cast<float>(fix(angular_, -1.0, 1.0));
        
        // (9) Publish velocity commands to Turtlebot
        // Set velocity command
        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = linear_;
        vel.twist.angular.z = angular_;

        // Publish velocity command
        vel_pub_->publish(vel);
    }

    // ROS Publisher, Subscribers, Timers
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
    bool have_odom_ = false;    // track first odom received
    bool have_scan_ = false;    // track first scan received
    rclcpp::Time last_check_time;
    double last_check_x_ = 0.0;
    double last_check_y_ = 0.0;
    bool init_progress_ = false;
    std::map<std::string, bool> bumpers_;
    float minLaserDist_;
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;
    float left_min_dist_ = std::numeric_limits<float>::infinity();
    float right_min_dist_ = std::numeric_limits<float>::infinity();
    float flip_direction = 1.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
