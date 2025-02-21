#ifndef TRAJECTORY_SAVER_HPP
#define TRAJECTORY_SAVER_HPP

// Necessary imports
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "amr_trajectory_tools/srv/save_trajectory_msg.hpp"
#include "json/json.h"
#include <fstream>
#include <vector>
#include <cmath>

// TrajectorySaver class
class TrajectorySaver : public rclcpp::Node {
public:
    TrajectorySaver();

private:
    // Declare parameters
    std::string odom_topic, marker_topic, frame;
    double update_frequency;
    
    // Store trajectory data (list of Pose messages)
    std::vector<nav_msgs::msg::Odometry::SharedPtr> trajectory;
    nav_msgs::msg::Odometry::SharedPtr latest_odom;
    
    // Subscribers, Publishers, and Services
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::Service<amr_trajectory_tools::srv::SaveTrajectoryMsg>::SharedPtr save_service;

    // Callback functions
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void update_trajectory();
    void publish_markers();
    void save_trajectory(const std::shared_ptr<amr_trajectory_tools::srv::SaveTrajectoryMsg::Request> request,
                         std::shared_ptr<amr_trajectory_tools::srv::SaveTrajectoryMsg::Response> response);
};

#endif // TRAJECTORY_SAVER_HPP

