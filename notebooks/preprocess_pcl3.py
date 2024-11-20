#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np
from functools import lru_cache

class PointCloudFilter:
    def __init__(self):
        # Initialize parameters
        self.height_threshold = -1.0
        self.distance_threshold = 30.0
        self.xy_threshold = 30.0
        self.ring_threshold = 80
        
        # Initialize the ROS node
        rospy.init_node('filtered_point_cloud_node', anonymous=True)
        
        # Pre-define the output fields for efficiency
        self.output_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
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
            self.point_cloud_callback,
            queue_size=1
        )
        
        rospy.loginfo("Point cloud filter node initialized")

    def point_cloud_callback(self, msg):
        """Optimized callback function to filter incoming point cloud data."""
        try:
            # Convert point cloud to numpy array more safely
            pointcloud = np.array(list(pc2.read_points(msg, skip_nans=True)))
            
            total_points = len(pointcloud)
            if total_points == 0:
                rospy.logwarn("Received empty point cloud")
                return

            # Apply ring filter using boolean indexing
            mask = pointcloud[:, 6] >= self.ring_threshold
            
            # Filter points and select columns
            filtered_points = np.zeros((np.sum(mask), 4), dtype=np.float32)
            filtered_points[:, :3] = pointcloud[mask, :3]  # xyz
            filtered_points[:, 3] = pointcloud[mask, 7]    # intensity
            
            # Create header
            header = std_msgs.msg.Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id
            
            # Convert to list for pc2.create_cloud
            filtered_points_list = filtered_points.tolist()
            
            # Create and publish filtered cloud
            filtered_msg = pc2.create_cloud(
                header,
                self.output_fields,
                filtered_points_list
            )
            
            self.filtered_pub.publish(filtered_msg)

            # Log filtering results
            filtered_count = len(filtered_points)
            rospy.loginfo(f"Filtered {filtered_count}/{total_points} points "
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