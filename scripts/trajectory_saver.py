#!/usr/bin/env python3

'''
# Author:    < Alen Gigi >
# Filename:  < trajectory_saver.py >
# Functions: < Record abd Save the trajecory of robot >
'''

#necesary imports
import rclpy
import json
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from amr_trajectory_tools.srv import SaveTrajectoryMsg
import math

#trajectry saver clas
class TrajectorySaver(Node):
    def __init__(self):

        # Initialize the node
        super().__init__('trajectory_saver_node')

        # Declare parameters with default values
        self.declare_parameter('pose_topic', '/odom')
        self.declare_parameter('marker_topic', '/trajectory_markers',)
        self.declare_parameter('frame', 'odom')
        self.declare_parameter('record_frequency', 5.0)

        # Get parameter values
        self.odom_topic = self.get_parameter('pose_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame = self.get_parameter('frame').value
        self.update_frequency = self.get_parameter('record_frequency').value

        # Store trajectory data (list of Pose messages)
        self.trajectory = []
        self.latest_odom = None  # Store latest received odometry message

        # Subscribe to the odometry topic
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Timer to update trajectory at a controlled frequency
        self.timer = self.create_timer(1.0 / self.update_frequency, self.update_trajectory)

        # Publisher for publishing trajectory data as marker arrays
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # Service to save trajectory data to a file
        self.save_service = self.create_service(SaveTrajectoryMsg, 'save_trajectory', self.save_trajectory)
        
        #some loggings..
        self.get_logger().info(f"Trajectory Saver Node Initialized.")
        self.get_logger().info(f"Subscribed to: {self.odom_topic}")
        self.get_logger().info(f"Publishing markers to: {self.marker_topic}")
        self.get_logger().info(f"Adding trajectory data at {self.update_frequency} Hz.")
        self.get_logger().info("To save a trajectory, call the service /save_trajectory with a filename (without extension) and duration in seconds. Data will be saved as a JSON file to your current directory.")

    def odom_callback(self, msg):
        """Callback function to Store the latest pose data"""
        self.latest_odom = msg  # Keep only the most recent message

    def update_trajectory(self):
        """Callback function to Add the pose data to the trajectory array."""
        if self.latest_odom:  # Add only if there's new data
            self.trajectory.append(self.latest_odom)
            self.publish_markers()

    def publish_markers(self):
        """Function to Publish the trajectory as a MarkerArray data."""
        marker_array = MarkerArray()

        for i, odom_msg in enumerate(self.trajectory):
            marker = Marker()
            marker.header.frame_id = self.frame #frame to publish the data
            marker.header.stamp = self.get_clock().now().to_msg() #get current time
            marker.ns = 'trajectory'
            marker.id = i
            marker.type = Marker.SPHERE #type of the marker
            marker.action = Marker.ADD
            marker.pose = odom_msg.pose.pose  # Extract pose from Odometry
            #dimensions of the marker x,y,z
            marker.scale.x = 0.05  
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            #color of the marker rgb and alpha (blue)
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.lifetime.sec = 10  # marker visibility time
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array) #publish the marker

    def save_trajectory(self, request, response):
        """Fuction to save the trajectory data to a Json file."""
        try:
            if not self.trajectory:
                response.success = False
                self.get_logger().warn("No trajectory data available to save.")
                return response

            latest_odom_msg = self.trajectory[-1] #get the last added pose data from trajectoy array
            current_time_ns = (latest_odom_msg.header.stamp.sec * 1e9) + latest_odom_msg.header.stamp.nanosec

            # Filter trajectory based on the requestd duration (backwards starting from current time)
            filtered_trajectory = [
                p for p in self.trajectory 
                if ((current_time_ns - ((p.header.stamp.sec * 1e9) + p.header.stamp.nanosec)) <= (request.duration * 1e9))
                and ((current_time_ns - ((p.header.stamp.sec * 1e9) + p.header.stamp.nanosec)) >= 0)  # Avoid future timestamps
            ]
            
            #extracting the x, y pose and yaw orientation of the robot
            trajectory_data = [
                {
                    'x': p.pose.pose.position.x,
                    'y': p.pose.pose.position.y,
                    'yaw': math.atan2(2.0 * (p.pose.pose.orientation.w * p.pose.pose.orientation.z),
                                    1.0 - 2.0 * (p.pose.pose.orientation.z ** 2)) # Convert quatrnion to yaw (2D orientaton)
                }
                for p in filtered_trajectory
            ]

            # Save data to file
            filename = f"{request.filename}.json"
            with open(filename, 'w') as file:
                json.dump(trajectory_data, file, indent=4)
            
            #service response for success or failure
            if trajectory_data:
                self.get_logger().info(f'Trajectory saved to {filename} with duration {request.duration} seconds')
                response.success = True
            else:
                self.get_logger().warn("Filtered trajectory is empty. No data saved.")
                response.success = False
        
        except Exception as e:
            self.get_logger().error(f'Failed to save trajectory: {str(e)}')
            response.success = False

        return response


def main(args=None):

    #initialize rclpy
    rclpy.init(args=args)
    #creating the node instance
    node = TrajectorySaver()
    #spin the node once
    rclpy.spin(node)
    #stop the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

