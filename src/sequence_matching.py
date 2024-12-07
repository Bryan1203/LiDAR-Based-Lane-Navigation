#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import time
import tf.transformations as tf_trans

class FrameSynchronizer:
    def __init__(self):
        rospy.init_node('frame_synchronizer', anonymous=True)
        
        # Buffer for point clouds with sequence as key
        self.odom_buffer = {}  # Dictionary with sequence number as key
        self.buffer_size = 500  # Maximum buffer size
        
        # Subscribe to point clouds first
        self.cloud_sub = rospy.Subscriber('/ouster/points_inference', PointCloud2, 
                                        self.cloud_callback, 
                                        queue_size=10)
        
        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber('/kiss/odometry', Odometry, 
                                       self.odom_callback, 
                                       queue_size=10)
        
        self.pub = rospy.Publisher('/synchronized_cloud_with_pose', PointCloud2, 
                                 queue_size=10)
        
        # Statistics
        self.total_cloud_msgs = 0
        self.total_odom_msgs = 0
        self.successful_matches = 0
        rospy.Timer(rospy.Duration(5.0), self.print_stats)

    def extract_sequence(self, msg):
        """Extract sequence number from child_frame_id"""
        if isinstance(msg, Odometry):
            # Extract sequence from child_frame_id
            try:
                return float(msg.child_frame_id)
            except (IndexError, ValueError):
                rospy.logwarn(f"Could not extract sequence from child_frame_id: {msg.child_frame_id}")
                return None
        else:
            # For PointCloud2 messages, extract sequence from the first point
            try:
                first_point = next(pc2.read_points(msg, skip_nans=True))
                return float(first_point[6])
            except Exception as e:
                rospy.logwarn(f"Could not extract sequence from point cloud: {e}")
                return None

    def transform_point(self, point, pose):
        """Transform a point using the pose from odometry"""
        # Extract translation and rotation from pose
        trans = pose.position
        rot = pose.orientation
        
        # Convert quaternion to rotation matrix
        q = [rot.x, rot.y, rot.z, rot.w]
        rotation_matrix = tf_trans.quaternion_matrix(q)[:3, :3]
        
        # Apply rotation and translation
        point_local = np.array([point[0], point[1], point[2]])
        point_global = np.dot(rotation_matrix, point_local) + np.array([trans.x, trans.y, trans.z])
        
        return point_global

    def odom_callback(self, odom_msg):
        """Store odometry messages in buffer"""
        self.total_odom_msgs += 1
        sequence = self.extract_sequence(odom_msg)
        if sequence is not None:
            self.odom_buffer[sequence] = odom_msg
            
            # Remove old messages if buffer is too large (may not work with looping of rosbag play, but should work in real-time)
            if len(self.odom_buffer) > self.buffer_size:
                oldest_seq = min(self.odom_buffer.keys())
                del self.odom_buffer[oldest_seq]

    def cloud_callback(self, cloud_msg):
        """Process each inference point cloud message"""
        self.total_cloud_msgs += 1
        
        # Extract sequence number from point cloud
        sequence = self.extract_sequence(cloud_msg)
        if sequence is None:
            return
            
        # Find matching odometry
        if sequence not in self.odom_buffer:
            rospy.logwarn(f"No matching odometry for sequence {sequence}")
            return

        odom_msg = self.odom_buffer[sequence]
        self.successful_matches += 1
        
        # Process point cloud with pose transformation
        cloud_points = []
        global_points = []
        predictions = []
        sequences = []
        poses = []
        try:
            for p in pc2.read_points(cloud_msg, skip_nans=True):
                # Transform point to global coordinates
                if p[4] < 0.5:
                    continue
                global_point = self.transform_point(p[:3], odom_msg.pose.pose)
                
                global_points.append(global_point)
                predictions.append(p[4])
                sequences.append(sequence)
                # poses.append(odom_msg.pose.pose)

                # cloud_points.append({
                #     # 'x': p[0],
                #     # 'y': p[1],
                #     # 'z': p[2],
                #     # 'intensity': p[3],
                #     'global_x': global_point[0],
                #     'global_y': global_point[1],
                #     'global_z': global_point[2],
                #     'prediction': p[4]
                # })

        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")
            return

        # Create output message
        # synchronized_msg = PointCloud2()
        # synchronized_msg.header = cloud_msg.header
        # synchronized_msg.header.stamp = odom_msg.header.stamp
        global_points = np.array(global_points)
        predictions = np.array(predictions)
        sequences = np.array(sequences)
        # poses = np.array(poses)
        # global_points_with_pred = np.column_stack((global_points, predictions))
        # fields = [
        #     # PointField('x', 0, PointField.FLOAT32, 1),
        #     # PointField('y', 4, PointField.FLOAT32, 1),
        #     # PointField('z', 8, PointField.FLOAT32, 1),
        #     # PointField('intensity', 12, PointField.FLOAT32, 1),
        #     PointField('global_x', 0, PointField.FLOAT32, 1),
        #     PointField('global_y', 4, PointField.FLOAT32, 1),
        #     PointField('global_z', 8, PointField.FLOAT32, 1),
        #     PointField('prediction', 12, PointField.FLOAT32, 1),
        #     # PointField('position_x', 16, PointField.FLOAT64, 1),
        #     # PointField('position_y', 24, PointField.FLOAT64, 1),
        #     # PointField('position_z', 32, PointField.FLOAT64, 1),
        #     # PointField('orientation_x', 40, PointField.FLOAT64, 1),
        #     # PointField('orientation_y', 48, PointField.FLOAT64, 1),
        #     # PointField('orientation_z', 56, PointField.FLOAT64, 1),
        #     # PointField('orientation_w', 64, PointField.FLOAT64, 1)
        # ]

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),  # RViz expects x,y,z as first fields
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('prediction', 12, PointField.FLOAT32, 1),
            PointField('sequences', 16, PointField.FLOAT32, 1)
        ]

        # Reuse the global coordinates as x,y,z (since that's what we want to visualize)
        global_points_with_pred = np.column_stack((
            global_points[:, 0],  # global_x becomes x
            global_points[:, 1],  # global_y becomes y
            global_points[:, 2],  # global_z becomes z
            predictions,
            sequences
        ))

        header = std_msgs.msg.Header()
        header.stamp = odom_msg.header.stamp
        header.seq = cloud_msg.header.seq
        header.frame_id = odom_msg.header.frame_id
        
        result_msg = pc2.create_cloud(header, fields, global_points_with_pred)
        self.pub.publish(result_msg)

        # synchronized_msg.point_step = 28
        # synchronized_msg.height = 1
        # synchronized_msg.width = len(cloud_points)
        # synchronized_msg.is_dense = False
        # synchronized_msg.row_step = synchronized_msg.point_step * synchronized_msg.width
        # synchronized_msg.is_bigendian = False

        # # Pack data
        # buffer = []
        # for point in cloud_points:
        #     buffer.extend([
        #         struct.pack('f', point['x']),
        #         struct.pack('f', point['y']),
        #         struct.pack('f', point['z']),
        #         struct.pack('f', point['intensity']),
        #         struct.pack('f', point['global_x']),
        #         struct.pack('f', point['global_y']),
        #         struct.pack('f', point['global_z'])
        #     ])
        
        # synchronized_msg.data = b''.join(buffer)
        # self.pub.publish(synchronized_msg)
        
        # Clean up used message from buffer
        del self.odom_buffer[sequence]

    def print_stats(self, event):
        if self.total_cloud_msgs > 0:
            match_rate = (self.successful_matches / float(self.total_cloud_msgs)) * 100
            
            rospy.loginfo(
                f"\nSync Statistics:\n"
                f"  Total Inference Messages: {self.total_cloud_msgs}\n"
                f"  Total Odometry Messages: {self.total_odom_msgs}\n"
                f"  Successful Matches: {self.successful_matches}\n"
                f"  Match Rate: {match_rate:.1f}%\n"
                f"  Odometry Buffer Size: {len(self.odom_buffer)}"
            )

if __name__ == '__main__':
    try:
        node = FrameSynchronizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass