#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>       // included for use of std::numeric_limits<float>::infinity();
#include <string>       // included for use of std::map<std::string, bool> bumpers_;
#include <functional>

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
        start_time_ = this->now();  // contest start time
        angular_ = 0.0;             // command angular velocity (rad/s)
        linear_ = 0.0;              // command linear velocity (m/s)

        // Odometry pose (stuck detection)
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;

        // Initialize bumper frames
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

        // Laser processing default values
        minLaserDist_ = std::numeric_limits<float>::infinity();   // min. distance in front sector
        left_min_dist_ = std::numeric_limits<float>::infinity();  // min. distance on left side
        right_min_dist_ = std::numeric_limits<float>::infinity(); // min. distance on right side
        nLasers_ = 0;           // total rays in scan
        desiredNLasers_ = 0;    // rays corresponding to the desiredAngle_
        desiredAngle_ = 15;     // checks +/- 15-degrees around forward (total sector width = 30-degrees)

        // Wall-following timers
        wall_follow_start_ = this->now();
        last_side_switch_time = this->now();

        // Contest Node start message:
        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");

    }

private:
    // Fix speed caps (helper function)
    static double fix(double speed, double low_speed, double high_speed) {
        return std::max(low_speed, std::min(speed, high_speed));
    }

    // Normalize angle to [-pi, pi]
    static double NormalizeAngle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < M_PI) a += 2.0 * M_PI;
        return a;
    }

    /*
    Choose which side seems more open (left or right)
        - Returns: +1 (left preferred) or -1 (right preferred). 
        - If both invalid (infinity/undefined) -> don't flip randomly but just give last direction. 
        - Alternate if both nearly equal
    */
    float choose_open_space(float left_d, float right_d) { 
        // If both invalid = do not randomly flip but keep last decision:
        if (!std::isfinite(left_d) && !std::isfinite(right_d)) {
            return flip_direction;    
        }
        // Prefer the side which has larger clearance:
        if (left_d > right_d + 0.001) {
            return 1.0;
        }
        if (right_d > left_d + 0.001) {
            return -1.0;
        }
        // If almost equal, choose alternate direction to avoid getting stuck choosing the same side:
        flip_direction *= -1.0;
        return flip_direction;
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        have_scan_ = true; // to prevent driving forward before scan arrives

        // For safety: if received empty scan or any invalid increment
        if (scan->ranges.empty() || scan->angle_increment <= 0.0) {
            return;
        }

        // Scan information
        nLasers_ = ((scan->angle_max - scan->angle_min) / scan->angle_increment);
        laserRange_ = scan->ranges;
        desiredNLasers_ = deg2rad(desiredAngle_) / scan->angle_increment; // corresponding rays
        
        // RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset: %d", nLasers_, desiredNLasers_);

        // LiDAR has 90-degree offset, so front of robot is at -90 degrees in scan frame
        float laser_offset = deg2rad(-90.0);
        uint32_t front_idx = (laser_offset - scan->angle_min) / scan->angle_increment;

        // Fix front_idx to be in the valid range
        front_idx = std::min(front_idx, static_cast<uint32_t>(laserRange_.size() - 1));

        // Reset distance for this scan
        minLaserDist_ = std::numeric_limits<float>::infinity();
        left_min_dist_ = std::numeric_limits<float>::infinity(); // Check for open space on left side
        right_min_dist_ = std::numeric_limits<float>::infinity(); // Check for open space on right side

        // Number of rays to sample on each side
        const uint32_t num_rays = static_cast<uint32_t>(desiredNLasers_);  // Number of laser rays to look on each side
        
        // Defining a center index for front_left and front_right:
        const uint32_t left_center = front_idx + num_rays; // Center index for left side
        uint32_t right_center; // Center index for right side
        if (front_idx > num_rays) {
            // To not go negative when shifting right (avoiding underflow when subtracting)
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

        // Find minimum laser distance within +/- desiredAngle from front center - main front collision distance
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
            // Scan everyhting safely if requested area is not valid
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

        have_odom_ = true;  // block startup motion until odom exists
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
        // Reset all bumpers to released state (not pressed)
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
            last_linear_ = 0.0;         // added for consistent memory update

            // Shutdown the node
            rclcpp::shutdown();
            return;
        }

        rclcpp::Time now = this->now();
        static bool force_recovery = false;
        static bool in_too_close = false;
        static rclcpp::Time too_close_start;

        // (1) Startup Robot Safety (do not move until /odom and /scan sensors are ready)
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
            last_linear_ = 0.0;
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
            last_linear_ = 0.0;
            return;
        }        

        // (2) Constants Fixed for Control loop (contest-safe) - tunable parameters
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
            // Recovery State:
        static bool recovering = false;
        static float recoveryTurnDirection = 1.0; // +1.0 left turn, -1.0 right turn
            // Turn Lock:
        static bool turn_lock = false;
        static rclcpp::Time turn_lock_start;
        static float turn_lock_dir = 1.0; // +1.0 left turn, -1.0 right turn
            // Odom-based Recovery:
        static int rec_stage = 0; // initialize (=0), reverse (=1), turn(=2), arc(=3)
        static double rec_start_x = 0.0;
        static double rec_start_y = 0.0;
        static double rec_target_yaw = 0.0;
        static rclcpp::Time rec_start_time;


        // (5) Stuck Detection:
            // If robot is commanded to go forward but is not moving (based on odom callback)
            // This forces recovery to escape trapped/enclosed areas or C-shaped/L-shaped obstacles
        const double check_period = 2.0; // time (in sec) between odom checks
        const double min_progress = 0.06; // 6cm
        const double want_forward = 0.08; // threshold for 'we intended to move forward'

        if (init_progress_) {
            double stuck_check_time = (now - last_check_time).seconds();

            if (stuck_check_time >= check_period) {
                double moved_dist = std::hypot(pos_x_ - last_check_x_, pos_y_ - last_check_y_); // distance moved is sqrt(dx^2 + dy^2) - diagonal between them

                // Update for next movement comparison
                last_check_time = now;
                last_check_x_ = pos_x_;
                last_check_y_ = pos_y_;

                bool near_obstacle = (minLaserDist_ < Slow_Dist);
                if (!recovering && !turn_lock && last_linear_ > want_forward && moved_dist < min_progress && near_obstacle) {
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
            wall_follow_active_ = false;
            force_recovery = false;
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
            
            rec_stage = 0; // Reset odom-based recovery state to start fresh
        }

        // (7) Recovery Mode & Normal Mode
        linear_ = 0.0;
        angular_ = 0.0;

        if (recovering) {
            const double back_dist = 0.28; // meters to reverse
            const double arc_dist = 0.22;  // meters to drive forward arc
            const double yaw_tol = deg2rad(7.0); // acceptable yaw tolerance or error
            const double timeout = 7.0; // max. time (sec) allowed in recovery

            // Initialize recovery stages once per recovery start
            if (rec_stage == 0) {
                rec_start_time = now;
                rec_stage = 1;

                rec_start_x = pos_x_;
                rec_start_y = pos_y_;

                // target is 90deg away from current yaw (turn direction decides sign)
                rec_target_yaw = NormalizeAngle(yaw_ + recoveryTurnDirection * deg2rad(90.0));
            }

            // If recovery takes too long, cancel it
            if ((now - rec_start_time).seconds() > timeout) {
                recovering = false;
                rec_stage = 0;
                linear_ = 0.0;
                angular_ = 0.0;
            }

            // Stage 1: reverse until backed up enough distance
            else if (rec_stage == 1) {
                double d = std::hypot(pos_x_ - rec_start_x, pos_y_ - rec_start_y);
                if (d < back_dist) {
                    linear_ = -0.10;
                    angular_ = 0.0;
                }
                else {
                    rec_stage = 2;
                }
            }

            // Stage 2: turn until yaw reaches target
            else if (rec_stage == 2) {
                double e = NormalizeAngle(rec_target_yaw - yaw_);
                if (std::fabs(e) > yaw_tol) {
                    linear_ = 0.0;
                    angular_ = Turn_Speed * recoveryTurnDirection;
                }
                else {
                    // reset distance baseline for arc stage
                    rec_start_x = pos_x_;
                    rec_start_y = pos_y_;
                    rec_stage = 3;
                }
            }

            // Stage 3: forward arc for a short distance
            else if (rec_stage == 3) {
                double d2 = std::hypot(pos_x_ - rec_start_x, pos_y_ - rec_start_y);
                if (d2 < arc_dist) {
                    linear_ = 0.08;
                    angular_ = 0.55 * recoveryTurnDirection;
                }
                else {
                    recovering = false;
                    rec_stage = 0;
                }
            }      
        }

        else {
            // Normal Navigation: forward drive + corner handling
            bool too_close = (minLaserDist_ < Stop_Dist);
            if (too_close) {
                wall_follow_active_ = false;

                // Corner time: if stay too_close for too long, force full recovery
                if (!in_too_close) {
                    in_too_close = true;
                    too_close_start = now;
                }
                if ((now - too_close_start).seconds() > 3.0) {
                    force_recovery = true;
                    in_too_close = false;
                }
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
                // Backup: unlock by time alone (1.2 sec) to prevent infinitely reverse spinning
                if (t_lock > 1.2) {
                    turn_lock = false;
                }
                // Reverse arc (better than spinning in-place in corners)
                linear_ = -0.10;
                angular_ = Turn_Speed * turn_lock_dir;     
            }
            else {
                in_too_close = false; // reset corner time whenever not too_close
                // Wall follow
                const float wall_follow_enter = 0.65; // start wall following if wall in this range
                const float wall_follow_exit = 0.95; // stop wall following if wall is far away
                const float desired_wall = 0.58; // target clearance to wall
                const float follow_time = 12.0; // time (sec) before switching sides to improve map coverage

                bool near_left = std::isfinite(left_min_dist_) && left_min_dist_ < wall_follow_enter;
                bool near_right = std::isfinite(right_min_dist_) && right_min_dist_ < wall_follow_enter;

                // Start wall follow if robot is near a wall
                if (!wall_follow_active_) {
                    if (near_left || near_right) {
                        wall_follow_active_ = true;
                        wall_follow_start_ = now;
                        // follow wall closer initially
                        if (near_left && !near_right) {
                            wall_follow_side_ = 1.0;
                        }
                        else if (!near_left && near_right) {
                            wall_follow_side_ = -1.0;
                        }
                        else {
                            if (left_min_dist_ <= right_min_dist_) {
                                wall_follow_side_ = 1.0;
                            }
                            else {
                                wall_follow_side_ = -1.0;
                            }
                        }
                    }
                }
                if (wall_follow_active_) {
                    float side_dist;
                    if (wall_follow_side_ > 0.0) {
                        side_dist = left_min_dist_;
                    }
                    else {
                        side_dist = right_min_dist_;
                    }

                    if (!std::isfinite(side_dist)) {
                        side_dist = desired_wall;
                    }
                    float diff_dist = side_dist - desired_wall;
                    float ang_cmd = diff_dist * wall_follow_side_;
                    ang_cmd = static_cast<float>(fix(ang_cmd, -0.6, 0.6));

                    if (minLaserDist_ < Slow_Dist) {
                        linear_ = Slow_Speed;
                    }
                    else {
                        linear_ = Fast_Speed;
                    }
                    angular_ = ang_cmd;
                    double t_follow = (now - wall_follow_start_).seconds();

                    if (t_follow > follow_time && (now - last_side_switch_time).seconds() > follow_time) {
                        wall_follow_side_ = -wall_follow_side_;
                        wall_follow_start_ = now;
                        last_side_switch_time = now;
                    }
                    bool wall_far = false;
                    if (side_dist > wall_follow_exit && minLaserDist_ > wall_follow_exit) {
                        wall_far = true;
                    }
                    if (wall_far) {
                        wall_follow_active_ = false;
                    }
                }

                else {
                    // Original forward + gentle steer
                    if (minLaserDist_ < Slow_Dist) {
                        linear_ = Slow_Speed;
                    }
                    else {
                        linear_ = Fast_Speed;
                    }
                    float steer_dir = 0.0;
                    if (left_min_dist_ < right_min_dist_ - 0.001) {
                        steer_dir = -1.0;
                    }
                    if (right_min_dist_ < left_min_dist_ - 0.001) {
                        steer_dir = 1.0;
                    }
                    angular_ = 0.35 * steer_dir;
                }
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

        last_linear_ = linear_;
    }

    // ROS Publisher, Subscribers, Timers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables:
    rclcpp::Time start_time_;
    float angular_;
    float linear_;

    // odom:
    double pos_x_;
    double pos_y_;
    double yaw_;

    // robot safety checks:
    bool have_odom_ = false;    // track first odom received
    bool have_scan_ = false;    // track first scan received

    // Stuck detection reference:
    rclcpp::Time last_check_time;
    double last_check_x_ = 0.0;
    double last_check_y_ = 0.0;
    bool init_progress_ = false;
    float last_linear_ = 0.0;

    // Bumpers:
    std::map<std::string, bool> bumpers_;

    // Laser processing:
    float minLaserDist_;
    int32_t nLasers_;
    int32_t desiredNLasers_;
    int32_t desiredAngle_;
    std::vector<float> laserRange_;
    float left_min_dist_ = std::numeric_limits<float>::infinity();
    float right_min_dist_ = std::numeric_limits<float>::infinity();
    float flip_direction = 1.0;

    // Wall following state:
    bool wall_follow_active_ = false;
    float wall_follow_side_ = 1.0; // 1.0 (follow left wall), -1.0 (follow right wall)
    rclcpp::Time wall_follow_start_;
    rclcpp::Time last_side_switch_time;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
