#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
from collections import deque

class SynchronizedDisplay:
    def __init__(self):
        rospy.init_node('synchronized_buffer', anonymous=True)
        self.queue = deque(maxlen=30)
        self.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('prediction', 12, PointField.FLOAT32, 1),
            PointField('sequences', 16, PointField.FLOAT32, 1)
        ]
        
        self.sync_sub = rospy.Subscriber('/synchronized_cloud_with_pose', PointCloud2, 
                                       self.sync_callback, queue_size=10)
        self.pub = rospy.Publisher('/synchronized_cloud_with_pose_buffer', PointCloud2, 
                                 queue_size=10)

        self.total_synchronized_messages = 0
        rospy.Timer(rospy.Duration(5.0), self.print_stats)

    def extract_sequence(self, msg):
        try:
            points = pc2.read_points(msg, field_names=("sequences",), skip_nans=True)
            return float(next(points)[0])
        except StopIteration:
            rospy.logwarn("Empty point cloud received")
            return None
        except Exception as e:
            rospy.logwarn(f"Error extracting sequence: {e}")
            return None
    
    def sync_callback(self, sync_msg):
        self.total_synchronized_messages += 1
        sequence = self.extract_sequence(sync_msg)
        
        if sequence is not None:
            # Convert message to point array for efficient storage
            points = np.array(list(pc2.read_points(sync_msg, skip_nans=True)))
            self.queue.append((sequence, points))
            
            # Combine all points from the queue
            all_points = np.vstack([points for _, points in self.queue])
            
            # Create and publish combined message
            header = std_msgs.msg.Header()
            header.stamp = sync_msg.header.stamp
            header.frame_id = sync_msg.header.frame_id
            
            result_msg = pc2.create_cloud(header, self.fields, all_points)
            self.pub.publish(result_msg)

    def print_stats(self, event):
        if self.total_synchronized_messages > 0:
            rospy.loginfo(
                f"Sync Statistics:\n"
                f"  Total Messages: {self.total_synchronized_messages}\n"
                f"  Queue Size: {len(self.queue)}"
            )

if __name__ == '__main__':
    try:
        node = SynchronizedDisplay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass