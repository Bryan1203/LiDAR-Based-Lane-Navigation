#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import message_filters
import struct
from collections import deque
import time
import tf

class FrameSynchronizer:
    def __init__(self):
       rospy.init_node('frame_synchronizer', anonymous=True)

       # Filtering Threshold
       self.ring_threshold = 80
       
       # Statistics
       self.total_msgs = 0
       self.last_publish_time = None
       self.publish_rate = []
       
       # Create subscribers using message_filters with tuned queue sizes
       cloud_sub = message_filters.Subscriber('/ouster/points', PointCloud2, queue_size=30)
       odom_sub = message_filters.Subscriber('/kiss/odometry', Odometry, queue_size=30)
       
       # Create approximate time synchronizer with tuned parameters
       self.ts = message_filters.ApproximateTimeSynchronizer(
           [cloud_sub, odom_sub],
           queue_size=30,    # 3 seconds worth of messages at 10 Hz
           slop=0.1,        # Just above the maximum observed period
           allow_headerless=False
       )
       
       # Register callback
       self.ts.registerCallback(self.sync_callback)
       
       # Publisher
       self.pub = rospy.Publisher('/synchronized_cloud_with_pose', PointCloud2, 
                                queue_size=30)
       
       rospy.Timer(rospy.Duration(5.0), self.print_stats)


    def sync_callback(self, cloud_msg, odom_msg):
        """Process synchronized messages"""
        self.total_msgs += 1
        
        # # Track publish rate
        # current_time = rospy.Time.now().to_sec()
        # if self.last_publish_time is not None:
        #     self.publish_rate.append(1.0 / (current_time - self.last_publish_time))
        #     if len(self.publish_rate) > 100:
        #         self.publish_rate.pop(0)
        # self.last_publish_time = current_time

        # Extract pose
        position = odom_msg.pose.pose.position
        orientation = odom_msg.pose.pose.orientation
        
        # Get full 4x4 transformation matrix
        transform_matrix = tf.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
        # Add translation
        transform_matrix[0:3, 3] = [position.x, position.y, position.z]

        # Process point cloud
        cloud_points = []
        try:
            for p in pc2.read_points(cloud_msg, skip_nans=True):
                if p[6] >= self.ring_threshold:
                    # Transform point to global coordinates using full matrix
                    local_point = np.array([p[0], p[1], p[2], 1])
                    global_point = np.dot(transform_matrix, local_point)[:3]
                    
                    cloud_points.append({
                        'x': global_point[0],
                        'y': global_point[1], 
                        'z': global_point[2],
                        'intensity': p[6]  # Fixed index
                    })
        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")
            return

        # Create output message
        synchronized_msg = PointCloud2()
        synchronized_msg.header = cloud_msg.header
        synchronized_msg.header.stamp = odom_msg.header.stamp
        
        synchronized_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1)
        ]
        
        synchronized_msg.point_step = 16
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
                struct.pack('f', point['intensity'])
            ])
        
        synchronized_msg.data = b''.join(buffer)
        self.pub.publish(synchronized_msg)


#    def sync_callback(self, cloud_msg, odom_msg):
#        """Process synchronized messages"""
#        self.total_msgs += 1
       
#        # Track publish rate
#        current_time = rospy.Time.now().to_sec()
#        if self.last_publish_time is not None:
#            self.publish_rate.append(1.0 / (current_time - self.last_publish_time))
#            if len(self.publish_rate) > 100:
#                self.publish_rate.pop(0)
#        self.last_publish_time = current_time

#        # Extract pose
#        position = odom_msg.pose.pose.position
       
#        # Process point cloud
#        cloud_points = []
#        try:
#            for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
#                cloud_points.append({
#                    'x': p[0],
#                    'y': p[1],
#                    'z': p[2],
#                    'intensity': p[3],
#                    'global_x': p[0] + position.x,
#                    'global_y': p[1] + position.y,
#                    'global_z': p[2] + position.z
#                })
#        except Exception as e:
#            rospy.logerr(f"Error processing point cloud: {e}")
#            return

#        # Create output message
#        synchronized_msg = PointCloud2()
#        synchronized_msg.header = cloud_msg.header
#        synchronized_msg.header.stamp = odom_msg.header.stamp
       
#        synchronized_msg.fields = [
#            PointField('x', 0, PointField.FLOAT32, 1),
#            PointField('y', 4, PointField.FLOAT32, 1),
#            PointField('z', 8, PointField.FLOAT32, 1),
#            PointField('intensity', 12, PointField.FLOAT32, 1),
#            PointField('global_x', 16, PointField.FLOAT32, 1),
#            PointField('global_y', 20, PointField.FLOAT32, 1),
#            PointField('global_z', 24, PointField.FLOAT32, 1),
#        ]
       
#        synchronized_msg.point_step = 28
#        synchronized_msg.height = 1
#        synchronized_msg.width = len(cloud_points)
#        synchronized_msg.is_dense = False
#        synchronized_msg.row_step = synchronized_msg.point_step * synchronized_msg.width
#        synchronized_msg.is_bigendian = False

#        # Pack data
#        buffer = []
#        for point in cloud_points:
#            buffer.extend([
#                struct.pack('f', point['x']),
#                struct.pack('f', point['y']),
#                struct.pack('f', point['z']),
#                struct.pack('f', point['intensity']),
#                struct.pack('f', point['global_x']),
#                struct.pack('f', point['global_y']),
#                struct.pack('f', point['global_z'])
#            ])
       
#        synchronized_msg.data = b''.join(buffer)
#        self.pub.publish(synchronized_msg)

    def print_stats(self, event):
       avg_publish_rate = sum(self.publish_rate) / len(self.publish_rate) if self.publish_rate else 0
       
       rospy.loginfo(
           f"\nSync Statistics:\n"
           f"  Total Synchronized Messages: {self.total_msgs}\n"
           f"  Average Publish Rate: {avg_publish_rate:.1f} Hz\n"
           f"  Time Difference Allowed: 0.2s\n"
           f"  Queue Size: 30"
       )

if __name__ == '__main__':
   try:
       node = FrameSynchronizer()
       rospy.spin()
   except rospy.ROSInterruptException:
       pass