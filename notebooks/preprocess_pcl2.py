#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np

class PointCloudFilter:
    def __init__(self):
        # Initialize parameters
        self.height_threshold = -1.0  # meters
        self.distance_threshold = 30.0  # meters
        self.xy_threshold = 30.0  # meters
        
        # Initialize the ROS node
        rospy.init_node('filtered_point_cloud_node', anonymous=True)
        
        # Create publisher for filtered point cloud
        self.filtered_pub = rospy.Publisher(
            'ouster/points_filtered', 
            PointCloud2, 
            queue_size=10
        )
        
        # Subscribe to raw point cloud
        self.sub = rospy.Subscriber(
            'ouster/points',
            PointCloud2,
            self.point_cloud_callback
        )
        
        rospy.loginfo("Point cloud filter node initialized")

    def point_cloud_callback(self, msg):
        """
        Callback function to filter incoming point cloud data.
        
        Args:
            msg (PointCloud2): Input point cloud message
        """
        try:
            # Convert point cloud to numpy array
            pointcloud = np.array(list(pc2.read_points(msg, skip_nans=True)))
            total_points = len(pointcloud)
            
            if total_points == 0:
                rospy.logwarn("Received empty point cloud")
                return
                
            # Extract coordinates and range
            x, y, z = pointcloud[:, 0], pointcloud[:, 1], pointcloud[:, 2]
            ring_values = pointcloud[:, 6]
            
            
            # Apply filtering conditions
            mask = (
                #(z <= self.height_threshold) &  # Height filter
                (ring_values >= 80)   # Distance filter
                #(np.abs(y) <= self.xy_threshold) &  # Y bounds filter
                #(np.abs(x) <= self.xy_threshold)  # X bounds filter
            )
            
            # Filter points
            
            filtered_points = pointcloud[mask]
            
            
            filtered_points = list(np.column_stack((filtered_points[:, :3], filtered_points[:, 7])))
            
            
            # Create filtered point cloud message
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = msg.header.frame_id
            
            # Use relevant fields from original message
            fields = [msg.fields[i] for i in [0, 1, 2, 3]]  # x, y, z, near_ir
      
            # Create and publish filtered cloud
            filtered_msg = pc2.create_cloud(header, fields, filtered_points)
            self.filtered_pub.publish(filtered_msg)
            
            # Log filtering results
            filtered_count = len(filtered_points)
            rospy.loginfo(f"Filtered {filtered_count}/{total_points} points " +
                         f"({filtered_count/total_points*100:.1f}%)")
            
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {str(e)}")

def main():
    try:
        node = PointCloudFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down point cloud filter node")

if __name__ == '__main__':
    main()