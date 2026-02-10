+#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>       // for infinity use
#include <string>       // for std::string

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

        // Subscriber for LiDAR scan
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::laserCallback, this, std::placeholders::_1));

        // Subscriber for hazard detections (includes bump)
        hazard_sub_ = this->create_subscription<irobot_create_msgs::msg::HazardDetectionVector>(
            "/hazard_detection", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::hazardCallback, this, std::placeholders::_1));

        // Subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Contest1Node::odomCallback, this, std::placeholders::_1));

        // Timer for main control loop at 10 Hz
        timer_ = this->create_wall_timer(
            100ms, std::bind(&Contest1Node::controlLoop, this));

        // Initialize variables
            // Record contest start time
        start_time_ = this->now();

            // Motion variables (published each loop)
        linear_ = 0.0;
        angular_ = 0.0;

            // Avoid movement before sensors arrive (startup safety)
        have_scan_ = false;
        have_odom_ = false;

            // Odom pose
        pos_x_ = 0.0;
        pos_y_ = 0.0;
        yaw_ = 0.0;

            // LiDAR variables
        nLasers_ = 0;
        desiredAngle_ = 15; // (FOV window around left/center/right)
        desiredNLasers_ = 0;
        center_distance_ = std::numeric_limits<float>::infinity();
        left_distance_ = std::numeric_limits<float>::infinity();
        right_distance_ = std::numeric_limits<float>::infinity();
        front_distance_ = std::numeric_limits<float>::infinity();

            // Bumper map keys
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;

            // Hazard tracking (for obstacles under LiDAR)
        hazard_active_ = false;
        last_hazard_time_ = this->now();

            // State machine initial state
        state_ = 0; // start in normal exploration state (=0)

            // Odom baseline (for fixed distance moves)
        start_pos_x_ = 0.0;
        start_pos_y_ = 0.0;
        have_start_pos_ = false;

            // Yaw baseline (for fixed angle turns)
        start_yaw_ = 0.0;
        have_start_yaw_ = false;

            // Recovery parameters (selected when entering recovery mode)
        turn_dir_ = 1;          // +1=left ; -1=right
        target_distance_ = 0.0; // meters (reverse)   
        target_rotation_ = 0.0; // radians (turn)
        target_arc_distance_ = 0.0; // meters (forward arc)

            // Stuck detection
        progress_init_ = false;
        last_progress_time_ = this->now();
        last_progress_x_ = 0.0;
        last_progress_y_ = 0.0;
        last_cmd_v_ = 0.0;

            // Recovery escalation
        recovery_level_ = 1;                    // Start at Level 1
        recovery_window_start_ = this->now();   // Recovery window start time
        recent_recovery_count_ = 0;             // Counter for recoveries inside window
        last_recovery_end_time_ = this->now();  // Last time we finished a recovery

            // Wall-following
        wall_follow_ = false;           // wall following (true/false)
        wall_side_ = 1;                 // +1 = left wall follow ; -1 = right wall follow
        wall_start_time_ = this->now(); // to switch sides occasionally

            // Periodic refresh (loop-breaker)
        last_refresh_time_ = this->now();
        refresh_dir_ = 1;
        refresh_target_rotation_ = deg2rad(80.0); // refresh turn amount (80-degrees)

            // Corner loop-breaker
        corner_window_start_ = this->now();
        corner_event_count_ = 0;

            // Turn lock to avoid jitter in corners:
        turn_lock_ = false;
        turn_lock_start_ = this->now();
        turn_lock_dir_ = 1;
        last_turn_dir_ = 1;

            // Spin-search (for strong Level 3 Recovery):
        spin_until_ = this->now();
      
        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    // Fix a number/command into a range
    double FixCmd(double vel, double low, double high) {
        if (vel < low) return low;
        if (vel > high) return high;
        return vel;
    }

    // Normalize angle to [-pi, +pi]
    double NormalizeAngle(double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // Choose open side based on LiDAR distances
    int ChooseOpenSide(float left_d, float right_d) {
        bool left_ok = std::isfinite(left_d);
        bool right_ok = std::isfinite(right_d);

        if (left_ok && right_ok) {
            if (left_d > right_d + 0.05f) return 1;  // left more open
            if (right_d > left_d + 0.05f) return -1; // right more open
            return last_turn_dir_;                  // equal = keep previous direction
        }
        if (left_ok && !right_ok) return 1;     // only left valid
        if (!left_ok && right_ok) return -1;    // only right valid
        return last_turn_dir_;                  // both invalid = keep previous direction
    }

    // Publish velocity command (making it 1 place for speed caps)
    void PublishCmd(double v, double w)
    {
        // Contest-safe speed caps
        v = FixCmd(v, -0.25, 0.25);
        w = FixCmd(w, -0.60, 0.60);

        geometry_msgs::msg::TwistStamped vel;
        vel.header.stamp = this->now();
        vel.twist.linear.x = v;
        vel.twist.angular.z = w;

        vel_pub_->publish(vel);

        last_cmd_v_ = v;    // remember last commanded forward speed
    }

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        have_scan_ = true;
        if (!scan) return;
        if (scan->ranges.empty()) return;
        if (scan->angle_increment <= 0.0) return;

        // Store ranges array and laser count
        laserRange_ = scan->ranges;         
        nLasers_ = (int)laserRange_.size(); // number of rays

        // If too small scan, do nothing
        if (nLasers_ < 10) return;

        // Create indices

        /* if LiDAR -90deg assumption is true, uncomment 
        and use front_angle_ in index calculations 
        
        double front_angle_ = deg2rad(-90.0);
        center_index : instead of 0.0 put front_angle_
        left_index : (front_angle + deg2rad(90.0))
        right_index : (front_angle - deg2rad(90.0))
        */
        //double front_angle_ = deg2rad(-90.0);    // LiDAR rotated front by -90deg
            // Center Index
        int center_index = (int)((0.0 - scan->angle_min) / scan->angle_increment);
        center_index = std::clamp(center_index, 0, nLasers_ - 1);
            // Left Index (+90deg from scan angles)
        int left_index = (int)((deg2rad(90.0) - scan->angle_min) / scan->angle_increment);
        left_index = std::clamp(left_index, 0, nLasers_ - 1);
            // Right Index (-90deg from scan angles)
        int right_index = (int)((deg2rad(-90.0) - scan->angle_min) / scan->angle_increment);
        right_index = std::clamp(right_index, 0, nLasers_ - 1);
        
        /*
        int center_index = nLasers_ / 2;     // straight ahead
        int left_index = nLasers_ * 3 / 4; // 90-deg left
        int right_index = nLasers_ / 4;      // 90-deg right
        */
        
        /*
        int center_index = (int)((0.0 - scan->angle_min) / scan->angle_increment);
        if (center_index < 0) center_index = 0;
        if (center_index >= nLasers_) center_index = nLasers_ - 1;

        int left_index = center_index + (nLasers_ / 4);
        int right_index = center_index - (nLasers_ / 4);

        if (left_index >= nLasers_) left_index = nLasers_ - 1;
        if (right_index < 0) right_index = 0;
        */


        // Convert desiredAngle_ into how many indices to check on each side
        desiredNLasers_ = (int)(deg2rad((double)desiredAngle_) / scan->angle_increment);
        if (desiredNLasers_ < 1) desiredNLasers_ = 1;

        // Take minimum within +/- desiredNLasers_ around each index (better than single ray)
        float cmin = std::numeric_limits<float>::infinity();
        float lmin = std::numeric_limits<float>::infinity();
        float rmin = std::numeric_limits<float>::infinity();

            // Center window:
        for (int i = center_index - desiredNLasers_; i <= center_index + desiredNLasers_; i++) {
            if (i >= 0 && i < nLasers_) {
                float v = laserRange_[i];
                if (std::isfinite(v) && v < cmin) cmin = v;
            }
        }

            // Left window:
        for (int i = left_index - desiredNLasers_; i <= left_index + desiredNLasers_; i++) {
            if (i >= 0 && i < nLasers_) {
                float v = laserRange_[i];
                if (std::isfinite(v) && v < lmin) lmin = v;
            }
        }

            // Right window:
        for (int i = right_index - desiredNLasers_; i <= right_index + desiredNLasers_; i++) {
            if (i >= 0 && i < nLasers_) {
                float v = laserRange_[i];
                if (std::isfinite(v) && v < rmin) rmin = v;
            }
        }

        int front_cone = desiredNLasers_ * 2; // 30-deg (desiredAngle_ = 15deg)
        if (front_cone < 1) front_cone = 1;

        float fmin = std::numeric_limits<float>::infinity();
        for (int i = center_index - front_cone; i <= center_index + front_cone; i++) {
            if (i >= 0 && i < nLasers_) {
                float v = laserRange_[i];
                if (std::isfinite(v) && v < fmin) fmin = v;
            }
        }
        front_distance_ = fmin;

        // Save into members used by Control Loop:
        center_distance_ = cmin;
        left_distance_ = lmin;
        right_distance_ = rmin;
        
        //RCLCPP_INFO(this->get_logger(), "LiDAR -> Center Distance: %.2f, Left Distance: %.2f, Right Distance = %.2f", center_distance_, left_distance_, right_distance_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        have_odom_ = true;
        if (!odom) return;

        // Save pose (position)
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        // Save yaw (orientation)
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        // Initialize stuck detection reference
        if (!progress_init_) {
            progress_init_ = true;
            last_progress_time_ = this->now();
            last_progress_x_ = pos_x_;
            last_progress_y_ = pos_y_;
        }
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        if (!hazard_vector) return;

        // Reset all bumpers to released state (not pressed)
        for (auto& [key, val] : bumpers_) {
            val = false;
        }

        // Default = no hazard
        hazard_active_ = false;

        // Update bumper states based on current detections
        for (const auto& detection : hazard_vector->detections) {
            // HazardDetection type: only physical bumper contact
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                hazard_active_ = true;
                last_hazard_time_ = this->now();    // latch timer
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

        // 1. Start-up safety
        if (!have_scan_ || !have_odom_) {
            PublishCmd(0.0, 0.0);
            return;
        }

        // 2. Constants (tunable parameters)
        double Stop_Dist = 0.50;    // if front closer than this -> escape corner/recovery
        double Slow_Dist = 0.75;    // slow down if front within this range
        double Fast_Speed = 0.25;   // contest maximum forward speed
        double Slow_Speed = 0.10;   // slow speed near obstacles

        double desired_wall = 0.62; // wall follow target distance
        double hazard_latch = 0.90; // hazard latch time (tiles under LiDAR)

        double refresh_period = 18.0; // do refresh turn every 18 sec
        double corner_window = 10.0;  // count corner events in this time frame
        int corner_max_events = 4;    // if too many corner issues (>4) -> force strong recovery (level 3)

        // 3. Hazard + Bumper Detection
        bool hazard_latched = ((now - last_hazard_time_).seconds() < hazard_latch);
        bool hazard = (hazard_active_ || hazard_latched);
        bool any_bumper_pressed = false;
        std::string pressed_bumper = "";
        // Iterate bumpers
        for (const auto &kv: bumpers_) {
            if (kv.second) {
                any_bumper_pressed = true;
                pressed_bumper = kv.first;
            }
        }

        // 4. Stuck Detection (odom)
        bool stuck = false;
        if (progress_init_) {
            double stuck_time = (now - last_progress_time_).seconds();
            if (stuck_time >= 2.0) {
                double moved = std::sqrt(std::pow(pos_x_ - last_progress_x_, 2) + std::pow(pos_y_ - last_progress_y_, 2));

                last_progress_time_ = now;
                last_progress_x_ = pos_x_;
                last_progress_y_ = pos_y_;

                // If commanded forward but moved almost nothing -> stuck detected
                if (last_cmd_v_ > 0.08 && moved < 0.05) {
                    stuck = true;
                }
            }
        }

        // 5. Corner counter reset
        if ((now - corner_window_start_).seconds() > corner_window) {
            corner_window_start_ = now;
            corner_event_count_ = 0;
        }

        // 6. Spin Search (If set spin_until, spin here for a short time)
        if (now < spin_until_) {
            linear_ = 0.0;
            angular_ = 0.55 * (double)turn_dir_;
            PublishCmd(linear_, angular_);
            return;
        }

        // 7. Periodic Refresh trigger (only when exploring)
        if (state_ == 0) {
            if ((now - last_refresh_time_).seconds() > refresh_period) {
                last_refresh_time_ = now;

                // alternate turn direction:
                if (refresh_dir_ == 1) refresh_dir_ = -1;
                else refresh_dir_ = 1;

                // go to refresh turn state
                state_ = 4;
                have_start_yaw_ = false;
                turn_dir_ = refresh_dir_;
                target_rotation_ = refresh_target_rotation_;

                linear_ = 0.0;
                angular_ = 0.0;

                PublishCmd(linear_, angular_);
                return;
            }
        }

        // Different States:

            // STATE 0 = NORMAL EXPLORATION
        if (state_ == 0) {

            // If hazard/bumper/stuck -> enter recovery
            if (hazard || any_bumper_pressed || stuck) {

                // Choose turn direction
                if (pressed_bumper.find("left") != std::string::npos) {
                    turn_dir_ = -1; // any left bumper hit -> turn right
                }
                else if (pressed_bumper.find("right") != std::string::npos) {
                    turn_dir_ = 1;  // any right bumper hit -> turn left
                }
                else { // front/center bumper -> decide using LiDAR
                    turn_dir_ = ChooseOpenSide(left_distance_, right_distance_);
                }

                last_turn_dir_ = turn_dir_;

                // Recovery Escalation logic
                    // Reset window every 20 sec
                    if ((now - recovery_window_start_).seconds() > 20.0) {
                        recovery_window_start_ = now;
                        recent_recovery_count_ = 0;
                    }
                    recent_recovery_count_ = recent_recovery_count_ + 1;

                    // Base Level
                    if (recent_recovery_count_ >= 3) recovery_level_ = 3;
                    else if (recent_recovery_count_ == 2) recovery_level_ = 2;
                    else recovery_level_ = 1;

                    // Escalate if got stuck again quickly after last recovery
                    if ((now - last_recovery_end_time_).seconds() < 8.0) {
                        if (recovery_level_ < 3) recovery_level_ = recovery_level_ + 1;
                    }

                    // Set recovery targets based on level
                    target_distance_ = 0.40;    // reverse distance
                    target_rotation_ = deg2rad(135.0); // turn angle
                    target_arc_distance_ = 0.30; // arc distance

                    if (recovery_level_ == 2) {
                        target_distance_ = 0.50;
                        target_rotation_ = deg2rad(170.0);
                        target_arc_distance_ = 0.40;
                    }

                    if (recovery_level_ == 3) {
                        target_distance_ = 0.65;
                        target_rotation_ = deg2rad(175.0);
                        target_arc_distance_ = 0.55;
                    }

                    // Start recovery at reverse stage
                    state_ = 1;
                    have_start_pos_ = false;
                    have_start_yaw_ = false;
                    wall_follow_ = false;

                    linear_ = 0.0;
                    angular_ = 0.0;

                    PublishCmd(linear_, angular_);
                    return;
            }
            // Front safety -> take min. of center/left/right
            double front_min = front_distance_;
            if (left_distance_ < front_min) front_min = left_distance_;
            if (right_distance_ < front_min) front_min = right_distance_;

            // Corner handling if front is too close
            if (front_min < Stop_Dist) {
                corner_event_count_ = corner_event_count_ + 1;

                // If too many corners -> force strong recovery
                if (corner_event_count_ >= corner_max_events) {
                    recovery_level_ = 3;

                    // Set recovery targets for level 3
                    target_distance_ = 0.65;
                    target_rotation_ = deg2rad(175.0);
                    target_arc_distance_ = 0.55;

                    turn_dir_ = ChooseOpenSide(left_distance_, right_distance_);
                    last_turn_dir_ = turn_dir_;

                    state_ = 1;
                    have_start_pos_ = false;
                    have_start_yaw_ = false;
                    wall_follow_ = false;

                    corner_event_count_ = 0;

                    linear_ = -0.12;
                    angular_ = 0.0;

                    PublishCmd(linear_, angular_);
                    return;
                }

                // Turn lock to avoid jitter
                if (!turn_lock_) {
                    turn_lock_ = true;
                    turn_lock_start_ = now;
                    turn_lock_dir_ = ChooseOpenSide(left_distance_, right_distance_);
                    last_turn_dir_ = turn_lock_dir_;
                }
                if ((now - turn_lock_start_).seconds() > 1.2) {
                    turn_lock_ = false;
                }
                linear_ = -0.12;
                angular_ = 0.45 * (double)turn_lock_dir_;
                PublishCmd(linear_, angular_);
                return;
            }

            // Clear turn lock after some time
            if (turn_lock_) {
                if ((now - turn_lock_start_).seconds() > 1.4) {
                    turn_lock_ = false;
                }
            }

            // Wall-follow starting conditions
            bool near_left = (left_distance_ < 0.70);
            bool near_right = (right_distance_ < 0.70);

            if (!wall_follow_) {
                if (near_left || near_right) {
                    wall_follow_ = true;
                    wall_start_time_ = now;

                    if (near_left && !near_right) wall_side_ = 1;
                    else if (!near_left && near_right) wall_side_ = -1;
                    else {
                        if (left_distance_ <= right_distance_) wall_side_ = 1;
                        else wall_side_ = -1;
                    }
                }
            }

            // Wall-follow behaviour
            if (wall_follow_) {
                double side_dist;
                if (wall_side_ == 1) side_dist = left_distance_;
                else side_dist = right_distance_;

                // Exit to open space (leave wall to map center area)
                if (side_dist > 1.20 && center_distance_ > 1.00) {
                    wall_follow_ = false;
                }
                else {
                    double err = side_dist - desired_wall;
                    double w = err * (double)wall_side_;
                    w = FixCmd(w, -0.55, 0.55);

                    double v = Fast_Speed;
                    if (front_distance_ < Slow_Dist) v = Slow_Speed;

                    // Switch sides occasionally
                    if ((now - wall_start_time_).seconds() > 12.0) {
                        wall_side_ = -wall_side_;
                        wall_start_time_ = now;
                    }
                    linear_ = v;
                    angular_ = w;
                    PublishCmd(linear_, angular_);
                    return;
                }
            }

            // Default wandering (open space bias)
            double v = Fast_Speed;
            if (front_distance_ < Slow_Dist) v = Slow_Speed;

            double w = 0.0;
            if (left_distance_ < right_distance_ - 0.07) w = -0.25;
            else if (right_distance_ < left_distance_ - 0.07) w = 0.25;
            else w = 0.0;

            linear_ = v;
            angular_ = w;
            PublishCmd(linear_, angular_);
            return;
        }

        // STATE 1 = RECOVERY: REVERSE FIXED DISTANCE
        if (state_ == 1) {
            // Record start position once
            if (!have_start_pos_) {
                start_pos_x_ = pos_x_;
                start_pos_y_ = pos_y_;
                have_start_pos_ = true;
            }

            double distance_traveled = std::sqrt(std::pow(pos_x_ - start_pos_x_, 2) + std::pow(pos_y_ - start_pos_y_, 2));

            if (distance_traveled < target_distance_) {
                linear_ = -0.12;
                angular_ = 0.0;
            }
            else {
                state_ = 2;
                have_start_pos_ = false;
                have_start_yaw_ = false;
                linear_ = 0.0;
                angular_ = 0.0;
            }

            PublishCmd(linear_, angular_);
            return;
        }

        // STATE 2 = RECOVERY: ROTATE FIXED YAW
        if (state_ == 2) {
            // Record start yaw once
            if (!have_start_yaw_) {
                start_yaw_ = yaw_;
                have_start_yaw_ = true;
            }

            double angle_rotated = std::fabs(NormalizeAngle(yaw_ - start_yaw_));

            if (angle_rotated < target_rotation_) {
                linear_ = 0.0;
                angular_ = 0.45 * (double)turn_dir_;
            }
            else {
                state_ = 3;
                have_start_yaw_ = false;
                have_start_pos_ = false;
                linear_ = 0.0;
                angular_ = 0.0;

                // If level 3 - short spin-search after turn
                if (recovery_level_ == 3) {
                    spin_until_ = this->now() + rclcpp::Duration::from_seconds(1.2);
                }
            }
            PublishCmd(linear_, angular_);
            return;
        }

        // STATE 3 = RECOVERY: FORWARD ARC FIXED DISTANCE
        if (state_ == 3) {
            // Record start position once
            if (!have_start_pos_) {
                start_pos_x_ = pos_x_;
                start_pos_y_ = pos_y_;
                have_start_pos_ = true;
            }

            double distance_traveled = std::sqrt(std::pow(pos_x_ - start_pos_x_, 2) + std::pow(pos_y_ - start_pos_y_, 2));

            if (distance_traveled < target_arc_distance_) {
                linear_ = 0.10;
                angular_ = 0.30 * (double)turn_dir_;
            }
            else {
                state_ = 0;
                have_start_pos_ = false;
                have_start_yaw_ = false;
                linear_ = 0.0;
                angular_ = 0.0;

                // Mark recovery end time for recovery escalation logic
                last_recovery_end_time_ = this->now();
            }
            PublishCmd(linear_, angular_);
            return;
        }

        // STATE 4 = REFRESH TURN (Improved coverage)
        if (state_ == 4) {
            // Record start yaw once
            if (!have_start_yaw_) {
                start_yaw_ = yaw_;
                have_start_yaw_ = true;
            }

            double angle_rotated = std::fabs(NormalizeAngle(yaw_ - start_yaw_));

            if (angle_rotated < target_rotation_) {
                linear_ = 0.0;
                angular_ = 0.35 * (double)turn_dir_;
            }
            else {
                state_ = 0;
                have_start_yaw_ = false;
                have_start_pos_ = false;
                linear_ = 0.0;
                angular_ = 0.0;
            }
            PublishCmd(linear_, angular_);
            return;
        }

        // Safety fallback
        PublishCmd(0.0, 0.0);
    }

    // ROS Objects
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<irobot_create_msgs::msg::HazardDetectionVector>::SharedPtr hazard_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Member variables
    rclcpp::Time start_time_;

    // Command variables
    double angular_;
    double linear_;

    // Startup safety
    bool have_scan_;
    bool have_odom_;

    // Odom pose
    double pos_x_;
    double pos_y_;
    double yaw_;

    // LiDAR variables
    std::vector<float> laserRange_;
    int nLasers_;
    int desiredNLasers_;
    int desiredAngle_;
    float center_distance_;
    float left_distance_;
    float right_distance_;
    float front_distance_;

    // Bumpers/hazards
    std::map<std::string, bool> bumpers_;
    bool hazard_active_;
    rclcpp::Time last_hazard_time_;

    // State machine
    int state_;

    // Odom reference
    double start_pos_x_;
    double start_pos_y_;
    bool have_start_pos_;
    double start_yaw_;
    bool have_start_yaw_;

    // Recovery parameters
    int turn_dir_;
    double target_distance_;
    double target_rotation_;
    double target_arc_distance_;

    // Stuck detection
    bool progress_init_;
    rclcpp::Time last_progress_time_;
    double last_progress_x_;
    double last_progress_y_;
    double last_cmd_v_;

    // Recovery escalation
    int recovery_level_;
    rclcpp::Time recovery_window_start_;
    int recent_recovery_count_;
    rclcpp::Time last_recovery_end_time_;

    // Wall follow
    bool wall_follow_;
    int wall_side_;
    rclcpp::Time wall_start_time_;

    // Refresh
    rclcpp::Time last_refresh_time_;
    int refresh_dir_;
    double refresh_target_rotation_;

    // Corner loop breaker
    rclcpp::Time corner_window_start_;
    int corner_event_count_;

    // Turn lock
    bool turn_lock_;
    rclcpp::Time turn_lock_start_;
    int turn_lock_dir_;
    int last_turn_dir_;

    // Spin search
    rclcpp::Time spin_until_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
