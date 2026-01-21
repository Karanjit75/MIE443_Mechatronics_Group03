#include <chrono>
#include <memory>
#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

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

// bumpers
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
        yaw_ 0.0;
        minLaserDist = std::numeric_limits<float>::infinity();
        nLasers_ = 0;
        desiredNLasers_ = 0;
        desiredAngle_ = 5;

        //  Initialize bumper states
        bumpers_["bump_front_left"] = false;
        bumpers_["bump_front_center"] = false;
        bumpers_["bump_front_right"] = false;
        bumpers_["bump_left"] = false;
        bumpers_["bump_right"] = false;
        RCLCPP_INFO(this->get_logger(), "Contest 1 node initialized. Running for 480 seconds.");
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // implement your code here
        nLasers_ = (scan->angle_max - scan ->angle_min) / scan ->angle_increment;
        laserRange_ = scan->ranges;
        desiredNLasers = deg2rad(desiredAngle_) / scan->angle_increment;
        RCLCPP_INFO(this->get_logger(), "Size of laser scan array: %d, and size of offset: %d", nLasers_, desiredNLasers_);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        // implement your code here
        // Extract position
        pos_x_ = odom->pose.pose.position.x;
        pos_y_ = odom->pose.pose.position.y;

        // Extract yaw from quaternion using tf2
        yaw_ = tf2::getYaw(odom->pose.pose.orientation);

        RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f), Orientation: %f rad or %f deg", pos_x_, pos_y_, yaw_, rad2deg(yaw_));
    }

    void hazardCallback(const irobot_create_msgs::msg::HazardDetectionVector::SharedPtr hazard_vector)
    {
        // implement your code here
        // Reset all bumpers to released state
        for (auto& [key, val] : bumpers_) {
            val = false;
        }

        // Update bumper states based on current detections
        for (const auto& detection : hazard_vector->detections) {
            // HazardDetection types include: BUMP, CLIFF, STALL, WHEEL_DROP, etc.
            // Type 1 corresponds to BUMP
            if (detection.type == irobot_create_msgs::msg::HazardDetection::BUMP) {
                bumpers_[detection.header.frame_id] = true;
                RCLCPP_INFO(this->get_logger(), "Bumper pressed: %s",
                            detection.header.frame_id.c_str());
            }
        }

    }

    /*void controlLoop()
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

        // Implement your exploration code here
        bool any_bumper_pressed = false;
        for (const auto& [key, val] : bumpers_) {
            if (val) { 
                any_bumper_pressed = true;
                break;
            }
        }
        
        if (pos_x_ > -0.5 && yaw_ < M_PI / 2 && !any_bumper_pressed) {
            angular_ = 0.0;
            linear_ = -0.2;
        }
        else if (yaw_ < M_PI / 2 && pos_x_ > -0.5 && !any_bumper_pressed) {
            angular_ = M_PI / 6;
            linear_ = 0.0;
        }
        else {
            angular_ = 0.0;
            linear_ = 0.0;
            rclcpp::shutdown();
            return;
        } 
    }*/

    /*
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
        
    // Stop if any bumper is pressed
    for (const auto& [key, val] : bumpers_) {
        if (val) {
            linear_ = 0.0;
            angular_ = 0.0;
            break;
        }
    }

    static bool moving_forward = true;
    static double start_x = pos_x_;
    static double start_y = pos_y_;
    static double start_yaw = yaw_;

    double distance_travelled = std::sqrt(
        std::pow(pos_x_ - start_x, 2) +
        std::pow(pos_y_ - start_y, 2)
    );

    // Move forward 0.5 m
    if (moving_forward && distance_travelled < 0.5) {
        linear_ = 0.2;
        angular_ = 0.0;
    }
    // Start turning
    else if (moving_forward) {
        moving_forward = false;
        start_yaw = yaw_;
        linear_ = 0.0;
        angular_ = M_PI / 6.0;
    }
    // Turn 90 degrees
    else {
        double yaw_diff = std::fabs(yaw_ - start_yaw);

        if (yaw_diff < M_PI / 2.0) {
            linear_ = 0.0;
            angular_ = M_PI / 6.0;
        } else {
            // Reset for next cycle
            moving_forward = true;
            start_x = pos_x_;
            start_y = pos_y_;
            linear_ = 0.0;
            angular_ = 0.0;
        }
    } */

    // ✅ Publish velocity command
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
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Contest1Node>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
