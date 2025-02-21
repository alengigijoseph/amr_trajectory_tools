#!/usr/bin/env python3

'''
# Author:    < Alen Gigi >
# Filename:  < trajectory_reader.py >
# Functions: < Publish the saved trajectory data for visualization >
'''

#necesary imports
import rclpy
import json
import math
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from amr_trajectory_tools.srv import ReadTrajectoryMsg  # Import the custom service
import math

#trajectory reader class
class TrajectoryReader(Node):
    def __init__(self):

        # Initialize the node
        super().__init__('trajectory_reader_node')

        # Declare parameters with default values
        self.declare_parameter(
            'marker_topic', '/read_trajectory_markers'
        )
        self.declare_parameter(
            'frame', 'odom'
        )

        # Get parameter values
        self.frame = self.get_parameter('frame').value
        self.marker_topic = self.get_parameter('marker_topic').value

        # Publisher for trajectory markers
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        # Service to trigger saved trajectory publishing
        self.load_service = self.create_service(ReadTrajectoryMsg , 'read_trajectory', self.read_trajectory)

        #some loggings..
        self.get_logger().info("Trajectory Reader Node Initialized.")
        self.get_logger().info("To Visualize a saved Trajectory, call the service /read_trajectory with the path of the Json file.")

    def read_trajectory(self, request, response):
        """Service callback to read given trajectory file and publish markers."""
        try:
            # Get filename from request
            file_path = request.filename
            
            #open the file in readmode
            with open(file_path, 'r') as file:
                trajectory_data = json.load(file)
            
            #handling case where the data is empty
            if not trajectory_data:
                self.get_logger().warn("Trajectory file is empty.")
                response.success = False
                response.message = "File is empty."
                return response
            
            #function call to publish the traectory data
            self.publish_markers(trajectory_data)
            self.get_logger().info(f"Trajectory loaded from {file_path} and published.")
            response.success = True
            response.message = f"Successfully loaded trajectory from {file_path}"
        except Exception as e:
            self.get_logger().error(f"Failed to load trajectory: {str(e)}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def publish_markers(self, trajectory_data):
        """Fuction to publish the trajectory as a MarkerArray."""
        marker_array = MarkerArray()

        for i, point in enumerate(trajectory_data):
            marker = Marker()
            marker.header.frame_id = self.frame  #frame to publish the data
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'loaded_trajectory'
            marker.id = i
            marker.type = Marker.SPHERE #type of the marker
            marker.action = Marker.ADD

            # Extract x, y from the saved trajectory
            marker.pose.position.x = point['x']
            marker.pose.position.y = point['y']

            # Convert yaw data back to quaternion
            qz = math.sin(point['yaw'] / 2)
            qw = math.cos(point['yaw'] / 2)
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            
            #dimensions of the marker x,y,z
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            
            #color of the marker rgb and alpha (green)
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.lifetime.sec = 0  # Keep markers permanently

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array) #publish the marker data
        self.get_logger().info("Trajectory markers published for visualization.")


def main(args=None):
    #initialize rclpy
    rclpy.init(args=args)
    #creating the node instance
    node = TrajectoryReader()
    #spin the node once
    rclpy.spin(node)
    #stop the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
