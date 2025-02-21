// trajectory_reader.hpp
#ifndef TRAJECTORY_READER_HPP
#define TRAJECTORY_READER_HPP

// Necessary imports
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <amr_trajectory_tools/srv/read_trajectory_msg.hpp>
#include <json/json.h>
#include <fstream>
#include <cmath>

// Trajectory reader class
class TrajectoryReader : public rclcpp::Node {
public:
    // Constructor
    TrajectoryReader();

private:
    // Service callback to read given trajectory file and publish markers
    void read_trajectory(
        const std::shared_ptr<amr_trajectory_tools::srv::ReadTrajectoryMsg::Request> request,
        std::shared_ptr<amr_trajectory_tools::srv::ReadTrajectoryMsg::Response> response);
    
    // Function to publish the trajectory as a MarkerArray
    void publish_markers(const Json::Value &trajectory_data);
    
    // Variables to store frame and topic name
    std::string frame_;
    std::string marker_topic_;
    
    // Publisher for trajectory markers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    // Service to trigger saved trajectory publishing
    rclcpp::Service<amr_trajectory_tools::srv::ReadTrajectoryMsg>::SharedPtr load_service_;
};

#endif // TRAJECTORY_READER_HPP