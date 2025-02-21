#include "amr_trajectory_tools/trajectory_saver.hpp"

/*
# Author:    < Alen Gigi >
# Filename:  < trajectory_saver.cpp >
# Functions: < Record and Save the trajectory of the robot >
*/

TrajectorySaver::TrajectorySaver() : Node("trajectory_saver_node") {
    
    // Declare parameters with default values
    this->declare_parameter<std::string>("pose_topic", "/odom");
    this->declare_parameter<std::string>("marker_topic", "/trajectory_markers");
    this->declare_parameter<std::string>("frame", "odom");
    this->declare_parameter<double>("record_frequency", 5.0);

    // Get parameter values
    odom_topic = this->get_parameter("pose_topic").as_string();
    marker_topic = this->get_parameter("marker_topic").as_string();
    frame = this->get_parameter("frame").as_string();
    update_frequency = this->get_parameter("record_frequency").as_double();

    // Subscribe to the odometry topic
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10, std::bind(&TrajectorySaver::odom_callback, this, std::placeholders::_1));
    
    // Timer to update trajectory at a controlled frequency
    timer = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency)),
        std::bind(&TrajectorySaver::update_trajectory, this));

    // Publisher for publishing trajectory data as marker arrays
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);

    // Service to save trajectory data to a file
    save_service = this->create_service<amr_trajectory_tools::srv::SaveTrajectoryMsg>(
        "save_trajectory", std::bind(&TrajectorySaver::save_trajectory, this, std::placeholders::_1, std::placeholders::_2));

    // Some log messages
    RCLCPP_INFO(this->get_logger(), "Trajectory Saver Node Initialized.");
    RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", odom_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing markers to: %s", marker_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "Adding trajectory data at %.2f Hz.", update_frequency);
    RCLCPP_INFO(this->get_logger(), "To save a trajectory, call the service /save_trajectory with a filename (without extension) and duration in seconds.");
}

void TrajectorySaver::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Callback function to store the latest pose data
    latest_odom = msg;
}

void TrajectorySaver::update_trajectory() {
    // Callback function to add the pose data to the trajectory array
    if (latest_odom) {
        trajectory.push_back(latest_odom);
        publish_markers();
    }
}

void TrajectorySaver::publish_markers() {
    // Function to publish the trajectory as a MarkerArray data
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < trajectory.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame; // Frame to publish the data
        marker.header.stamp = this->now(); // Get current time
        marker.ns = "trajectory";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE; // Type of the marker
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = trajectory[i]->pose.pose; // Extract pose from Odometry
        
        // Dimensions of the marker x, y, z
        marker.scale.x = 0.05;  
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        // Color of the marker RGB and alpha (red)
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker.lifetime = rclcpp::Duration::from_seconds(10); // Marker visibility time
        marker_array.markers.push_back(marker);
    }

    marker_pub->publish(marker_array); // Publish the marker
}

void TrajectorySaver::save_trajectory(const std::shared_ptr<amr_trajectory_tools::srv::SaveTrajectoryMsg::Request> request,
                                      std::shared_ptr<amr_trajectory_tools::srv::SaveTrajectoryMsg::Response> response) {
    // Function to save the trajectory data to a file
    try {
        if (trajectory.empty()) {
            response->success = false;
            RCLCPP_WARN(this->get_logger(), "No trajectory data available to save.");
            return;
        }

        auto latest_odom_msg = trajectory.back();
        auto current_time_ns = latest_odom_msg->header.stamp.sec * 1e9 + latest_odom_msg->header.stamp.nanosec;

        std::vector<nav_msgs::msg::Odometry::SharedPtr> filtered_trajectory;
        for (const auto &p : trajectory) {
            auto timestamp_ns = p->header.stamp.sec * 1e9 + p->header.stamp.nanosec;
            if (current_time_ns - timestamp_ns <= request->duration * 1e9 && current_time_ns - timestamp_ns >= 0) {
                filtered_trajectory.push_back(p);
            }
        }

        // Save trajectory to JSON file
        Json::Value trajectory_data;
        for (const auto &p : filtered_trajectory) {
            Json::Value entry;
            entry["x"] = p->pose.pose.position.x;
            entry["y"] = p->pose.pose.position.y;
            entry["yaw"] = std::atan2(2.0 * (p->pose.pose.orientation.w * p->pose.pose.orientation.z),
                                      1.0 - 2.0 * (p->pose.pose.orientation.z * p->pose.pose.orientation.z));
            trajectory_data.append(entry);
        }

        std::ofstream file(request->filename + ".json");
        file << trajectory_data;

        response->success = !filtered_trajectory.empty();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Trajectory saved to %s.json", request->filename.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Filtered trajectory is empty. No data saved.");
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save trajectory: %s", e.what());
        response->success = false;
    }
}

int main(int argc, char **argv) {
    // Main function to initialize and spin the ROS2 node
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectorySaver>());
    rclcpp::shutdown();
    return 0;
}