#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import message_filters
import struct

class FrameSynchronizer:
    def __init__(self):
        rospy.init_node('frame_synchronizer', anonymous=True)
        
        # Statistics
        self.total_msgs = 0
        self.last_publish_time = None
        self.publish_rate = []
        
        # Create subscribers using message_filters
        cloud_sub = message_filters.Subscriber('/ouster/points', PointCloud2)
        odom_sub = message_filters.Subscriber('/kiss/odometry', Odometry)
        
        # Create approximate time synchronizer
        # Parameters: 
        # - queue size (10)
        # - maximum allowed time difference (0.1 seconds)
        # - topics to synchronize
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [cloud_sub, odom_sub],
            queue_size=10,
            slop=0.1
        )
        
        # Register callback
        self.ts.registerCallback(self.sync_callback)
        
        # Publisher
        self.pub = rospy.Publisher('/synchronized_cloud_with_pose', PointCloud2, 
                                 queue_size=10)
        
        rospy.Timer(rospy.Duration(5.0), self.print_stats)

    def sync_callback(self, cloud_msg, odom_msg):
        """Process synchronized messages"""
        self.total_msgs += 1
        
        # Track publish rate
        current_time = rospy.Time.now().to_sec()
        if self.last_publish_time is not None:
            interval = current_time - self.last_publish_time
            self.publish_rate.append(1.0/interval)
            if len(self.publish_rate) > 100:
                self.publish_rate.pop(0)
        self.last_publish_time = current_time
        
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
        avg_publish_rate = sum(self.publish_rate) / len(self.publish_rate) if self.publish_rate else 0
        rospy.loginfo(
            f"\nSync Statistics:\n"
            f"  Total Synchronized Messages: {self.total_msgs}\n"
            f"  Average Publish Rate: {avg_publish_rate:.1f} Hz"
        )

if __name__ == '__main__':
    try:
        node = FrameSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass