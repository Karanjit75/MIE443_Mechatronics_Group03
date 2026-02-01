#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>
#include <limits>       // included for use of std::numeric_limits<float>::infinity();
#include <string>       // included for use of std::map<std::string, bool> bumpers_;
#include <cstdlib>      // for rand() use in direction change for recovery

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

        // Find minimum laser distance within +/- desiredAngle from front center
        if (deg2rad(desiredAngle_) < scan->angle_max && deg2rad(desiredAngle_) > scan->angle_min) {
            for (uint32_t laser_idx = front_idx - desiredNLasers_; laser_idx < front_idx + desiredNLasers_; ++laser_idx) {
                minLaserDist_ = std::min(minLaserDist_, laserRange_[laser_idx]);
            }
        } else {
            for (uint32_t laser_idx = 0; laser_idx < nLasers_; ++laser_idx) {
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

        /*
        Control Loop Strategy:
            1) If any_bumper_pressed = true -> recover (reverse for a bit and then turn away)
            2) Else if minLaserDist_ is too small -> turn in place
            3) Else -> drive forward
            4) If near obstacles -> speed capped at 0.10 m/s
            5) Speed cap always to 0.25 m/s */
       
        // Don't move until odom has arrived at least once (for robot safety)
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

        // Don't move until at least one scan arrives (prevents blindly driving forward during start up)
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

        // Constants Fixed for Control loop (contest-safe)
        const float Stop_Dist = 0.30;  // If obstacle closer than 0.3 m, turn
        const float Slow_Dist = 0.50;  // If near obstacle zone, speed <= 0.10 m/s
        const float Fast_Speed = 0.25; // <=0.25 m/s maximum
        const float Slow_Speed = 0.10; // slow-zone speed cap (<=0.10 m/s)
        const float Turn_Speed = 0.60; // Turning speed (in rad/s)

        // Bumper States
        bool LeftPressed = bumpers_["bump_left"];
        bool RightPressed = bumpers_["bump_right"];
        bool CenterPressed = bumpers_["bump_front_center"] || bumpers_["bump_front_left"] || bumpers_["bump_front_right"];
        bool any_bumper_pressed = LeftPressed || RightPressed || CenterPressed;

        // Recovery memory across loops
        static bool recovering = false;
        static rclcpp::Time recoverStart;
        static float recoveryTurnDirection = 1.0; // +1.0 left turn, -1.0 right turn

        // If we hit something, start Recovery state
        if (any_bumper_pressed && !recovering) {
            recovering = true;
            recoverStart = this->now();

            // Choose turn direction based on which bumper was hit
            if (LeftPressed) {
                recoveryTurnDirection = -1.0;    // If Left bumper hit -> turn right
            }
            else if (RightPressed) {
                recoveryTurnDirection = 1.0;     // If Right bumper hit -> turn left 
            }
            else {
                if ((rand() % 2) == 0) {
                    // If Center bumper hit -> flip a coin to eventually find open space on left or right turn
                    recoveryTurnDirection = 1.0;
                }
                else {
                    recoveryTurnDirection = -1.0;
                }    
            }
            RCLCPP_WARN(this->get_logger(), "Recovery has started. Turn direction = %.0f", recoveryTurnDirection);
        }

        if (recovering) {
            double recovery_time = (this->now() - recoverStart).seconds();

            if (recovery_time < 0.6) {
                // To gently reverse the robot
                linear_ = -0.06;
                angular_ = 0.0;
            }
            else if (recovery_time < 1.8) {
                // Turn in place away from contact
                linear_ = 0.0;
                angular_ = Turn_Speed * recoveryTurnDirection;
            }
            else {
                // Recovery finished
                recovering = false;
                linear_ = 0.0;
                angular_ = 0.0;
            }
        }
        else {
            // Normal exploration done by robot based on minLaserDist_
            if (minLaserDist_ < Stop_Dist) {
                // Too close then rotate away
                linear_ = 0.0;
                // Randomized turn direction to avoid looping the same wrong turn
                if ((rand() % 2) == 0) {
                    angular_ = Turn_Speed;  // left turn (CCW)
                }
                else {
                    angular_ = -Turn_Speed; // right turn (CW)
                } 
            }
            else {
                // If safe, then drive forward. Slow down near obstacles.
                if (minLaserDist_ < Slow_Dist) {
                    linear_ = Slow_Speed;
                    angular_ = 0.0;
                }
                else {
                    linear_ = Fast_Speed;
                    angular_ = 0.0;
                }
            }
        }

        // Stuck Detection:
            // If robot is commanded to go forward but is not moving (based on odom callback)
            // This forces recovery to escape trapped/enclosed areas or C-shaped/L-shaped obstacles

            const double check_period = 2.0; // time (in sec) between odom checks
            const double min_progress = 0.05; // 5cm
            const double want_forward = 0.05; // threshold for 'we intended to move forward'

            if (init_progress_) {
                double check_time = (this->now() - last_check_time).seconds();

                if (check_time >= check_period) {
                    double moved_dist = std::hypot(pos_x_ - last_check_x_, pos_y_ - last_check_y_); // distance moved is sqrt(dx^2 + dy^2) - diagonal between them

                    // Update for next movement comparison
                    last_check_time = this->now();
                    last_check_x_ = pos_x_;
                    last_check_y_ = pos_y_;

                    // Triggers in Normal mode (not already recovering)
                    if (!recovering && linear_ > want_forward && moved_dist < min_progress) {
                        RCLCPP_WARN(this->get_logger(), "Stuck detected: moved %.4f m in %.1f sec. Forcing recovery now.", moved_dist, check_period);

                        // Start forced recovery (same as bumper recovery)
                        recovering = true;
                        recoverStart = this->now();

                        // Pick direction to escape to (randomized left/right to help with symmetric enclosed areas like C-shapes)
                        if ((rand() % 2) == 0) {
                            recoveryTurnDirection = 1.0;    
                        }
                        else {
                            recoveryTurnDirection = -1.0;
                        }

                        // Stop any forward motion in this cycle (recovery can handle later cycles)
                        linear_ = 0.0;
                        angular_ = 0.0;
                    }
                }
            }

        // Contest 1 Speed Caps
            // Hard speed cap:
        linear_ = static_cast<float>(fix(linear_, -0.25, 0.25));

            // Slow zone (obstacles) speed cap:
        if (minLaserDist_ < Slow_Dist) {
            linear_ = static_cast<float>(fix(linear_, -0.10, 0.10));
        }

            // Angular speed cap (to prevent violent spinning of the robot):
        angular_ = static_cast<float>(fix(angular_, -1.0, 1.0));
        
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}