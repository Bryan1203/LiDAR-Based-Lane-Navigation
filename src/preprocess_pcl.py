#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np
from functools import lru_cache
import time

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
            start_time = time.time()
            filtered = []
            for point in pc2.read_points(msg, skip_nans=True):
                if point[6]>=self.ring_threshold:
                    filtered.append((point[0],point[1],point[2],point[6]))

            post_filter_time = time.time()
            # Create header
            header = std_msgs.msg.Header()
            header.stamp = msg.header.stamp
            header.frame_id = msg.header.frame_id

            # Create and publish filtered cloud
            filtered_msg = pc2.create_cloud(
                header,
                self.output_fields,
                filtered
            )

            self.filtered_pub.publish(filtered_msg)

            end_time = time.time()

            # Log Time
            rospy.loginfo(f"Filter Time {post_filter_time - start_time}\n"
                          f"Total Time {end_time - start_time}")

            # # Log filtering results
            # filtered_count = len(filtered_points)
            # rospy.loginfo(f"Filtered {filtered_count}/{total_points} points "
            #             f"({filtered_count/total_points*100:.1f}%)")

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