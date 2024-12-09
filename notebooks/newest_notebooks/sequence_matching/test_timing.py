#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
from collections import deque
import time

class FrameSynchronizer:
    def __init__(self):
        rospy.init_node('frame_synchronizer', anonymous=True)
        
        # Timing tracking
        self.last_cloud_time = None
        self.last_odom_time = None
        self.last_point_cloud = None
        
        # Debug counters
        self.cloud_count = 0
        self.odom_count = 0
        
        # Statistics for periodic reporting
        self.cloud_processing_delays = []
        self.odom_processing_delays = []
        rospy.Timer(rospy.Duration(10.0), self.print_stats)
        
        # Subscribe to point clouds first
        rospy.loginfo("Subscribing to point cloud topic...")
        self.cloud_sub = rospy.Subscriber('/ouster/points', PointCloud2, 
                                        self.cloud_callback, 
                                        queue_size=100)
        
        rospy.sleep(2.0)  # Wait for point cloud subscriber to initialize
        
        # Subscribe to odometry
        rospy.loginfo("Subscribing to odometry topic...")
        self.odom_sub = rospy.Subscriber('/kiss/odometry', Odometry, 
                                       self.odom_callback, 
                                       queue_size=10)

    def cloud_callback(self, cloud_msg):
        cloud_time = cloud_msg.header.stamp.to_sec()
        current_time = rospy.Time.now().to_sec()
        self.cloud_count += 1
        
        # Calculate processing delay
        processing_delay = current_time - cloud_time
        self.cloud_processing_delays.append(processing_delay)
        
        if self.last_cloud_time:
            interval = cloud_time - self.last_cloud_time
            rospy.loginfo(f"\nCloud #{self.cloud_count}:"
                         f"\n  Timestamp: {cloud_time:.6f}"
                         f"\n  Current Time: {current_time:.6f}"
                         f"\n  Age of data: {processing_delay:.6f}s"
                         f"\n  Interval between messages: {interval:.6f}s")
        
        self.last_cloud_time = cloud_time
        self.last_point_cloud = cloud_msg

    def odom_callback(self, odom_msg):
        odom_time = odom_msg.header.stamp.to_sec()
        current_time = rospy.Time.now().to_sec()
        self.odom_count += 1

        # Calculate processing delay
        processing_delay = current_time - odom_time
        self.odom_processing_delays.append(processing_delay)

        # Find which point cloud this odometry came from
        if self.last_point_cloud:
            source_cloud_time = self.last_point_cloud.header.stamp.to_sec()
            kiss_processing_time = odom_time - source_cloud_time
            
            rospy.loginfo(f"\nOdom #{self.odom_count}:"
                         f"\n  Odom timestamp: {odom_time:.6f}"
                         f"\n  Source cloud time: {source_cloud_time:.6f}"
                         f"\n  Current Time: {current_time:.6f}"
                         f"\n  KISS-ICP processing time: {kiss_processing_time:.6f}s"
                         f"\n  Total delay: {current_time - source_cloud_time:.6f}s")

            if self.last_odom_time:
                interval = odom_time - self.last_odom_time
                rospy.loginfo(f"  Interval between odom messages: {interval:.6f}s")

        self.last_odom_time = odom_time

    def print_stats(self, event):
        """Print statistics every 10 seconds"""
        if not self.cloud_processing_delays or not self.odom_processing_delays:
            return

        # Calculate cloud statistics
        avg_cloud_delay = np.mean(self.cloud_processing_delays)
        min_cloud_delay = np.min(self.cloud_processing_delays)
        max_cloud_delay = np.max(self.cloud_processing_delays)
        
        # Calculate odom statistics
        avg_odom_delay = np.mean(self.odom_processing_delays)
        min_odom_delay = np.min(self.odom_processing_delays)
        max_odom_delay = np.max(self.odom_processing_delays)

        rospy.loginfo(
            f"\n=== Processing Statistics ==="
            f"\nPoint Cloud Messages:"
            f"\n  Total received: {self.cloud_count}"
            f"\n  Average delay: {avg_cloud_delay:.6f}s"
            f"\n  Min delay: {min_cloud_delay:.6f}s"
            f"\n  Max delay: {max_cloud_delay:.6f}s"
            f"\nOdometry Messages:"
            f"\n  Total received: {self.odom_count}"
            f"\n  Average delay: {avg_odom_delay:.6f}s"
            f"\n  Min delay: {min_odom_delay:.6f}s"
            f"\n  Max delay: {max_odom_delay:.6f}s"
        )

        # Keep only recent delays for memory efficiency
        if len(self.cloud_processing_delays) > 1000:
            self.cloud_processing_delays = self.cloud_processing_delays[-1000:]
        if len(self.odom_processing_delays) > 1000:
            self.odom_processing_delays = self.odom_processing_delays[-1000:]

if __name__ == '__main__':
    try:
        node = FrameSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass