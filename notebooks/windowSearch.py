import numpy as np
import rospy
from sklearn.cluster import DBSCAN
import std_msgs.msg
from collections import Counter, deque
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import math


class WaypointGenerator:
    def __init__(self, frame_buffer_size=20):
        # Initialize parameters
        rospy.init_node('waypoint_generator', anonymous=True)

        self.odom_buffer = {}  # Dictionary with sequence number as key
        self.buffer_size = 1500  # Maximum buffer size

        # Subscribe to odometry
        self.odom_sub = rospy.Subscriber('/kiss/odometry', Odometry, 
                                       self.odom_callback, 
                                       queue_size=100)

        self.frame_buffer = deque(maxlen=frame_buffer_size)
        self.pcl_sub = rospy.Subscriber('/synchronized_cloud_with_pose', PointCloud2, self.pcl_callback)
        self.rightlane_pub = rospy.Publisher(
            'ouster/points_rightlane',
            PointCloud2,
            queue_size=10
        )
        self.buffer_pub = rospy.Publisher('buffer_pcl', PointCloud2, queue_size=10)
        self.line_pub = rospy.Publisher('lane_marker_topic', Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber('/kiss/odometry', Odometry, self.odom_callback, queue_size=100)
        self.position = np.zeros(3)
        self.orientation = np.zeros(4)

        self.marker_pub = rospy.Publisher('/waypoint_marker', Marker, queue_size=1)
        self.window_pub = rospy.Publisher('/window_bound', Marker, queue_size=1)

    def get_yaw_from_quaternion(self, orientation):
        # Convert quaternion to Euler angles
        siny_cosp = 2 * (orientation[3] * orientation[2] + orientation[0] * orientation[1])
        cosy_cosp = 1 - 2 * (orientation[1] * orientation[1] + orientation[2] * orientation[2])
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def is_point_right(point, robot_pose):
        # Create robot's heading vector
        heading = np.array([
            np.cos(robot_pose[2]),
            np.sin(robot_pose[2])
        ])
        
        # Vector from robot to point
        to_point = point - robot_pose[:2]
        
        # Cross product (in 2D)
        cross_product = heading[0] * to_point[1] - heading[1] * to_point[0]
        
        return False if cross_product > 0 else True

    def extract_sequence(self, msg):
        """Extract sequence number from child_frame_id"""
        if isinstance(msg, Odometry):
            # Extract sequence from child_frame_id
            try:
                return int(msg.child_frame_id)
            except (IndexError, ValueError):
                rospy.logwarn(f"Could not extract sequence from child_frame_id: {msg.child_frame_id}")
                return None
        else:
            # For PointCloud2 messages, extract sequence from the first point
            try:
                first_point = next(pc2.read_points(msg, skip_nans=True))
                return int(first_point[6])
            except Exception as e:
                rospy.logwarn(f"Could not extract sequence from point cloud: {e}")
                return None
        
    def odom_callback(self, odom_msg):
        """Store odometry messages in buffer"""
        sequence = self.extract_sequence(odom_msg)
        if sequence is not None:
            self.odom_buffer[sequence] = odom_msg
            rospy.loginfo(f"Stored odometry message with sequence {sequence}")
            # Remove old messages if buffer is too large (may not work with looping of rosbag play, but should work in real-time)
            if len(self.odom_buffer) > self.buffer_size:
                oldest_seq = min(self.odom_buffer.keys())
                del self.odom_buffer[oldest_seq]

    def pointcloud2_to_array(self, cloud_msg):
        try:
            pc_data = pc2.read_points(cloud_msg, field_names=("x", "y", "z", "prediction","sequences"), skip_nans=True)
            return np.array(list(pc_data))
        except Exception as e:
            rospy.logerr(f"Error converting PointCloud2 to array: {e}")
            return np.array([])

    def local_to_global(self, x_robot, y_robot, theta, window_x_bound, window_y_bound):
        rotation_matrix = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        lower_left_local = np.array([window_x_bound[0], window_y_bound[0]])
        upper_right_local = np.array([window_x_bound[1], window_y_bound[1]])
        lower_left_global = np.dot(rotation_matrix, lower_left_local) + np.array([x_robot, y_robot])
        upper_right_global = np.dot(rotation_matrix, upper_right_local) + np.array([x_robot, y_robot])
        return lower_left_global, upper_right_global

    def pcl_callback(self, msg):
        try:
            cloud_array = self.pointcloud2_to_array(msg)
            if cloud_array.size == 0:
                rospy.logwarn("Empty point cloud received.")
                return

            points = np.zeros((cloud_array.shape[0], 3))
            points[:, 0] = cloud_array[:, 0]  # x
            points[:, 1] = cloud_array[:, 1]  # y
            points[:, 2] = cloud_array[:, 2]  # z
            
            sequence = cloud_array[0, 4]    # sequence number
            odom_msg = self.odom_buffer.get(sequence)
            if odom_msg is None:
                rospy.logwarn(f"No matching odometry found for sequence {sequence}")
                return
            # Extract lane points
            lane_line_points = points[cloud_array[:, 3] == 1]
            self.frame_buffer.append(lane_line_points)
            lane_line_points = np.vstack(list(self.frame_buffer))
            # Publish buffer as PointCloud2
            self.publish_filtered_cloud(msg, lane_line_points, self.buffer_pub)


            # Clustering with DBSCAN
            lane_xy = lane_line_points[:, :2]
            dbscan = DBSCAN(eps=1.0, min_samples=10)
            labels = dbscan.fit_predict(lane_xy)


            self.position = np.array([odom_msg.pose.pose.position.x,
                                    odom_msg.pose.pose.position.y,
                                    odom_msg.pose.pose.position.z])
            self.orientation = np.array([odom_msg.pose.pose.orientation.x,
                                        odom_msg.pose.pose.orientation.y,
                                        odom_msg.pose.pose.orientation.z,
                                        odom_msg.pose.pose.orientation.w])

            # Get robot pose
            x_robot = self.position[0]
            y_robot = self.position[1]
            theta = self.get_yaw_from_quaternion(self.orientation)

            # Create rotation matrix (use regular theta, not negative)
            rotation_matrix = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]
            ])

            translation_vector = np.array([x_robot, y_robot])

            min_distance = 1000
            right_lane_label = 0
            square_indices = []
            local_lane_points = np.zeros_like(lane_line_points)
            for i in range(len(labels)):
                point_global = lane_line_points[i, :2]
                # First subtract translation, then rotate with transpose matrix
                point_local = np.dot(rotation_matrix.T, (point_global - translation_vector))
                local_lane_points[i, :2] = point_local
                dist = (point_local[1])
                if point_local[1] < -0.3 and point_local[1] > -3 and abs(point_local[0]) < 3 and point_local[0] > 0 and dist < min_distance:
                    min_distance = dist
                    right_lane_label = labels[i]
            # Filter points for the most common cluster
            cluster_mask = labels == right_lane_label
            right_lane_points = lane_line_points[cluster_mask]

            # Publish filtered cluster as PointCloud2
            self.publish_filtered_cloud(msg, right_lane_points, self.rightlane_pub)

            self.find_waypoints(msg, local_lane_points, x_robot, y_robot, theta)

        except Exception as e:
            rospy.logerr(f"Error processing point cloud: {e}")

    def find_waypoints(self, original_msg, local_lane_points, x_robot, y_robot, theta):
        window_x_bound = np.array([2.1, 2.4])
        window_y_bound = np.array([-3.0, 0])
        window_height = 0.3
        window_shift = 0.0
        steering_angle = []
        avg_points = []
        window_bound = []
        for i in range(60):
            within_x = np.logical_and(local_lane_points[:, 0] > window_x_bound[0], local_lane_points[:, 0] < window_x_bound[1])
            within_y = np.logical_and(local_lane_points[:, 1] > window_y_bound[0], local_lane_points[:, 1] < window_y_bound[1])
            within_window = np.logical_and(within_x, within_y)
            window_points = local_lane_points[within_window]
            if(len(window_points) < 3):
                window_x_bound -= window_height
                window_y_bound += window_shift
                break
            avg_x = np.mean(window_points[:, 0])
            avg_y = np.mean(window_points[:, 1])
            window_x_bound += window_height
            window_shift = avg_y-np.mean(window_y_bound)
            window_y_bound += window_shift
            steering_angle.append(math.degrees(math.atan2(window_shift,window_height)))
            shifted_x = avg_x - 1.6*np.sin(np.radians(steering_angle[-1]))
            shifted_y = avg_y + 1.6*np.cos(np.radians(steering_angle[-1]))
            # Create rotation matrix (use regular theta, not negative)
            rotation_matrix = np.array([
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta), np.cos(theta)]
            ])
            # Transform from local frame to global frame
            shifted_point_local = np.array([shifted_x, shifted_y])
            shifted_point_global = np.dot(rotation_matrix, shifted_point_local) + np.array([x_robot, y_robot])
            if i!=0:
                avg_points.append(shifted_point_global)
                for i in range(2):
                    for j in range(2):
                        window_local = np.array([window_x_bound[i], window_y_bound[j]])
                        window_global = np.dot(rotation_matrix, window_local) + np.array([x_robot, y_robot])
                        window_bound.append(window_global)

        rospy.loginfo(f'Steering Angle: {steering_angle}')
        self.publish_point_markers(original_msg, window_bound, self.window_pub)
        self.publish_point_markers(original_msg, avg_points, self.marker_pub)
        
    def publish_point_markers(self, original_msg, points, publisher):
        """Publish visualization markers for average points"""
        marker = Marker()
        marker.header.frame_id = original_msg.header.frame_id
        marker.header.stamp = original_msg.header.stamp
        marker.ns = "lane_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Set marker color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        # Add points
        marker.points = []
        for point in points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0  # Or point[2] if you have z coordinates
            marker.points.append(p)
        
        publisher.publish(marker)
        return


    def publish_filtered_cloud(self, original_msg, points, publisher):
        try:
            if points.shape[0] == 0:
                rospy.logwarn("No points to publish.")
                return

            header = std_msgs.msg.Header()
            header.stamp = original_msg.header.stamp
            header.frame_id = original_msg.header.frame_id

            fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            filtered_msg = pc2.create_cloud(header, fields, points.tolist())
            publisher.publish(filtered_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing filtered cloud: {e}")


if __name__ == '__main__':
    try:
        node = WaypointGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
