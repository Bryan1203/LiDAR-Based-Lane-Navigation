#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import struct
from collections import deque
import time

class FrameSynchronizer:
    def __init__(self):
        rospy.init_node('frame_synchronizer', anonymous=True)
        
        # Buffer for point clouds with timestamp as key
        self.cloud_buffer = deque(maxlen=100)  # Adjust based on memory constraints
        self.adaptive_threshold = 0.3   # start with 0.3s threshold
        self.max_threshold = 0.5    # maximum allowable threshold

        self.time_differences = []  # debug info

        # Subscribe to point clouds first with larger queue
        self.cloud_sub = rospy.Subscriber('/ouster/points', PointCloud2, 
                                        self.cloud_callback, 
                                        queue_size=100)
        
        # Small delay to let point cloud buffer populate
        rospy.sleep(2.0)
        
        # Now subscribe to odometry - this will drive our processing
        self.odom_sub = rospy.Subscriber('/kiss/odometry', Odometry, 
                                       self.odom_callback, 
                                       queue_size=10)
        
        self.pub = rospy.Publisher('/synchronized_cloud_with_pose', PointCloud2, 
                                 queue_size=10)
        
        # Statistics
        self.total_odom_msgs = 0
        self.successful_matches = 0
        rospy.Timer(rospy.Duration(5.0), self.print_stats)

    def cloud_callback(self, cloud_msg):
        """Just store point clouds in buffer"""
        self.cloud_buffer.append((cloud_msg.header.stamp.to_sec(), cloud_msg))

    def find_closest_cloud(self, odom_time):
        """Find the closest point cloud to given odometry timestamp"""
        if not self.cloud_buffer:
            return None
            
        closest_msg = None
        min_diff = float('inf')
    
        # Get time range info
        cloud_times = [t for t, _ in self.cloud_buffer]
        buffer_start = min(cloud_times)
        buffer_end = max(cloud_times)
        
        # Check if odom time is within buffer range
        if odom_time < buffer_start - self.adaptive_threshold or odom_time > buffer_end + self.adaptive_threshold:
            rospy.logwarn(f"Odom time {odom_time} outside buffer range [{buffer_start}, {buffer_end}]")
            return None
        
        for cloud_time, cloud_msg in self.cloud_buffer:
            time_diff = abs(odom_time - cloud_time)
            if time_diff < min_diff:
                min_diff = time_diff
                closest_msg = cloud_msg

        # Store time difference for adaptive threshold
        self.time_differences.append(min_diff)
        if len(self.time_differences) > 50:  # Keep last 50 differences
            self.time_differences.pop(0)

        # Update adaptive threshold based on recent history
        if len(self.time_differences) > 10:
            median_diff = sorted(self.time_differences)[len(self.time_differences)//2]
            self.adaptive_threshold = min(self.max_threshold, median_diff * 1.5)
        
        
        # Only use if within reasonable time difference
        if min_diff > self.adaptive_threshold:  # 300ms threshold, adjust as needed
            rospy.logwarn(f"Closest point cloud is {min_diff:.3f}s away from odometry")
            return None
            
        return closest_msg

    def odom_callback(self, odom_msg):
        """Process each odometry message"""
        self.total_odom_msgs += 1
        odom_time = odom_msg.header.stamp.to_sec()
        
        # Find matching point cloud
        cloud_msg = self.find_closest_cloud(odom_time)
        if cloud_msg is None:
            rospy.logwarn(f"No matching point cloud for odometry at time {odom_time:.3f}")
            return

        self.successful_matches += 1
        
        # Extract pose
        position = odom_msg.pose.pose.position
        
        # Process point cloud
        cloud_points = []
        try:
            for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
                cloud_points.append({
                    'x': p[0],
                    'y': p[1],
                    'z': p[2],
                    'intensity': p[3],
                    'global_x': p[0] + position.x,
                    'global_y': p[1] + position.y,
                    'global_z': p[2] + position.z
                })
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")
            return

        # Create output message
        synchronized_msg = PointCloud2()
        synchronized_msg.header = cloud_msg.header
        # Also store odometry timestamp for reference
        synchronized_msg.header.stamp = odom_msg.header.stamp
        
        synchronized_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('global_x', 16, PointField.FLOAT32, 1),
            PointField('global_y', 20, PointField.FLOAT32, 1),
            PointField('global_z', 24, PointField.FLOAT32, 1),
        ]
        
        synchronized_msg.point_step = 28
        synchronized_msg.height = 1
        synchronized_msg.width = len(cloud_points)
        synchronized_msg.is_dense = False
        synchronized_msg.row_step = synchronized_msg.point_step * synchronized_msg.width
        synchronized_msg.is_bigendian = False

        # Pack data
        buffer = []
        for point in cloud_points:
            buffer.extend([
                struct.pack('f', point['x']),
                struct.pack('f', point['y']),
                struct.pack('f', point['z']),
                struct.pack('f', point['intensity']),
                struct.pack('f', point['global_x']),
                struct.pack('f', point['global_y']),
                struct.pack('f', point['global_z'])
            ])
        
        synchronized_msg.data = b''.join(buffer)
        self.pub.publish(synchronized_msg)

    def print_stats(self, event):
        if self.total_odom_msgs > 0:
            match_rate = (self.successful_matches / float(self.total_odom_msgs)) * 100
            avg_time_diff = sum(self.time_differences) / len(self.time_differences) if self.time_differences else 0
            
            rospy.loginfo(
                f"\nSync Statistics:\n"
                f"  Total Odometry Messages: {self.total_odom_msgs}\n"
                f"  Successful Matches: {self.successful_matches}\n"
                f"  Match Rate: {match_rate:.1f}%\n"
                f"  Point Cloud Buffer Size: {len(self.cloud_buffer)}\n"
                f"  Current Threshold: {self.adaptive_threshold:.3f}s\n"
                f"  Average Time Difference: {avg_time_diff:.3f}s"
            )

if __name__ == '__main__':
    try:
        node = FrameSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass