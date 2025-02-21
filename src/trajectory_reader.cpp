#include "amr_trajectory_tools/trajectory_reader.hpp"

/*
# Author:    < Alen Gigi >
# Filename:  < trajectory_reader.cpp >
# Functions: < Publish the saved trajectory data for visualization >
*/

// Initialize the node
TrajectoryReader::TrajectoryReader() : Node("trajectory_reader_node") {
    // Declare parameters with default values
    declare_parameter("marker_topic", "/read_trajectory_markers");
    declare_parameter("frame", "odom");

    // Get parameter values
    frame_ = get_parameter("frame").as_string();
    marker_topic_ = get_parameter("marker_topic").as_string();

    // Publisher for trajectory markers
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    // Service to trigger saved trajectory publishing
    load_service_ = create_service<amr_trajectory_tools::srv::ReadTrajectoryMsg>(
        "read_trajectory", std::bind(&TrajectoryReader::read_trajectory, this, std::placeholders::_1, std::placeholders::_2));

    // Some loggings
    RCLCPP_INFO(get_logger(), "Trajectory Reader Node Initialized.");
    RCLCPP_INFO(get_logger(), "To visualize a saved trajectory, call the service /read_trajectory with the path of the JSON file.");
}

void TrajectoryReader::read_trajectory(
    const std::shared_ptr<amr_trajectory_tools::srv::ReadTrajectoryMsg::Request> request,
    std::shared_ptr<amr_trajectory_tools::srv::ReadTrajectoryMsg::Response> response) {
    try {
        // Get filename from request
        std::ifstream file(request->filename);
        Json::Value trajectory_data;
        file >> trajectory_data;

        // Handling case where the data is empty
        if (trajectory_data.empty()) {
            RCLCPP_WARN(get_logger(), "Trajectory file is empty.");
            response->success = false;
            response->message = "File is empty.";
            return;
        }

        // Function call to publish the trajectory data
        publish_markers(trajectory_data);
        RCLCPP_INFO(get_logger(), "Trajectory loaded from %s and published.", request->filename.c_str());
        response->success = true;
        response->message = "Successfully loaded trajectory.";
    } catch (const std::exception &e) {
        RCLCPP_ERROR(get_logger(), "Failed to load trajectory: %s", e.what());
        response->success = false;
        response->message = "Error loading trajectory.";
    }
}

void TrajectoryReader::publish_markers(const Json::Value &trajectory_data) {
    // Function to publish the trajectory as a MarkerArray
    visualization_msgs::msg::MarkerArray marker_array;

    for (Json::ArrayIndex i = 0; i < trajectory_data.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_; // Frame to publish the data
        marker.header.stamp = now();
        marker.ns = "loaded_trajectory";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::SPHERE; // Type of the marker
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Extract x, y from the saved trajectory
        marker.pose.position.x = trajectory_data[i]["x"].asFloat();
        marker.pose.position.y = trajectory_data[i]["y"].asFloat();

        // Convert yaw data back to quaternion
        float yaw = trajectory_data[i]["yaw"].asFloat();
        marker.pose.orientation.z = std::sin(yaw / 2);
        marker.pose.orientation.w = std::cos(yaw / 2);

        // Dimensions of the marker x, y, z
        marker.scale.x = marker.scale.y = marker.scale.z = 0.08;

        // Color of the marker (green)
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_array.markers.push_back(marker);
    }

    marker_pub_->publish(marker_array); // Publish the marker data
    RCLCPP_INFO(get_logger(), "Trajectory markers published for visualization.");
}

int main(int argc, char *argv[]) {
    // Initialize rclcpp
    rclcpp::init(argc, argv);
    // Create the node instance
    auto node = std::make_shared<TrajectoryReader>();
    // Spin the node once
    rclcpp::spin(node);
    // Stop the node
    rclcpp::shutdown();
    return 0;
}