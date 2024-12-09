#!/usr/bin/env python3

from __future__ import print_function
import os, csv, math, threading
import numpy as np
from numpy import linalg as la
import scipy.signal as signal
import rospy
from std_msgs.msg import String, Bool, Float32, Float64
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped

class PID(object):
    def __init__(self, kp, ki, kd, wg=None):
        """Initialize PID controller with gains and windup guard"""
        self.iterm = 0
        self.last_t = None
        self.last_e = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.wg = wg
        self.derror = 0

    def reset(self):
        """Reset controller states"""
        self.iterm = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):
        """Calculate PID control value based on error"""
        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)
        
        # Anti-windup
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de

class OnlineFilter(object):
    def __init__(self, cutoff, fs, order):
        """Initialize Butterworth filter"""
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        """Filter incoming data"""
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted

class PurePursuit(object):
    def __init__(self):
        self.rate = rospy.Rate(10)
        self.look_ahead = 5  # Look-ahead distance
        self.wheelbase = 1.8  # Vehicle wheelbase

        # Path storage and threading lock
        self.path_lock = threading.Lock()
        self.current_path_x = []  # Path points in vehicle's forward direction
        self.current_path_y = []  # Path points in vehicle's right direction
        self.wp_size = 0
        self.dist_arr = np.array([])
        self.last_goal_index = 0
        
        # Subscribe to topics
        self.waypoints_sub = rospy.Subscriber("/waypoint_marker", Marker, self.waypoints_callback, queue_size=1)
        self.global_pose_sub = rospy.Subscriber("/waypoint/pose", Odometry, self.global_pose_callback, queue_size=1)
        self.speed_sub = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)
        self.used_waypoints_pub = rospy.Publisher("/used_waypoints", Marker, queue_size=1)
        # Initialize the publisher
        self.pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)
        # Vehicle state
        self.speed = 0.0
        self.gem_enable = False
        self.pacmod_enable = False  
        
        # Speed control parameters
        self.desired_speed = 1
        self.max_accel = 0.5
        self.pid_speed = PID(0.5, 0.0, 0.1, wg=20)
        self.speed_filter = OnlineFilter(1.2, 30, 4)

        # Initialize PACMod control
        self.setup_pacmod_control()

        self.kevin_msg = None

    def setup_pacmod_control(self):
        """Initialize PACMod publishers and messages"""
        # Enable control
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # Gear control
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2  # SHIFT_NEUTRAL

        # Brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear = True
        self.brake_cmd.ignore = True

        # Acceleration control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear = True
        self.accel_cmd.ignore = True

        # Turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1  # None

        # Steering control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0
        self.steer_cmd.angular_velocity_limit = 3.5

    def waypoints_callback(self, msg):
        """Process incoming waypoints marker message"""
        # Save message for visualization
        self.kevin_msg = msg
        # rospy.loginfo(msg.pose)
        if msg.type != msg.POINTS:
            rospy.logwarn("Invalid marker type. Expected POINTS.")
            return
            
        # with self.path_lock:
        new_path_x = []
        new_path_y = []
        # Convert points to vehicle frame (x-forward, y-right)
        for point in msg.points:
            new_path_x.append(point.x)
            new_path_y.append(-point.y)
        
        # Smooth transition between path updates
        if self.current_path_x:
            closest_idx = self.find_closest_point(
                self.current_path_x[self.last_goal_index],
                self.current_path_y[self.last_goal_index],
                new_path_x,
                new_path_y
            )
            self.last_goal_index = closest_idx
        
        self.current_path_x = new_path_x
        self.current_path_y = new_path_y
        self.wp_size = len(self.current_path_x)
        self.dist_arr = np.zeros(self.wp_size)

    def global_pose_callback(self, msg):
        """Update current vehicle position"""
        # rospy.loginfo("Current pose: %s", msg.pose.pose)
        self.current_pose = msg.pose.pose
        # Create a PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = msg.header.stamp
        pose_msg.header.frame_id = msg.header.frame_id
        pose_msg.pose = self.current_pose

        # Publish the pose
        self.pose_pub.publish(pose_msg)


    def speed_callback(self, msg):
        """Update current vehicle speed"""
        self.speed = round(msg.vehicle_speed, 3)

    def enable_callback(self, msg):
        """Update PACMod enable status"""
        # rospy.loginfo("PACMod enable status: %s", msg.data)
        self.pacmod_enable = msg.data

    def find_closest_point(self, x, y, path_x, path_y):
        """Find closest point index in path to given point"""
        min_dist = float('inf')
        closest_idx = 0
        for i in range(len(path_x)):
            d = self.dist((path_x[i], path_y[i]), (x, y))
            if d < min_dist:
                min_dist = d
                closest_idx = i
        return closest_idx

    def find_angle(self, v1, v2):
        """Find angle between two vectors"""
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)

    def dist(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def front2steer(self, f_angle):
        """Convert front wheel angle to steering angle"""
        if(f_angle > 35): f_angle = 35
        if(f_angle < -35): f_angle = -35
        if(f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        elif(f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0
        return steer_angle
    
    def publish_point_markers(self, original_msg, points, publisher):
        """Publish visualization markers for average points"""
        marker = Marker()
        marker.header.frame_id = original_msg.header.frame_id
        marker.header.stamp = original_msg.header.stamp
        marker.ns = "used_waypoints"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        
        # Set marker properties
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Set marker color (green)
        marker.color.r = 1.0
        marker.color.g = 0.0
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

    def start_pp(self):
        """Main pure pursuit control loop"""
        while not rospy.is_shutdown():
            # Initialize PACMod if not enabled
            if not self.gem_enable and self.pacmod_enable:
                self.enable_pacmod()
                
            # with self.path_lock:

            # print length of current path
            rospy.loginfo("Current path length: %d", len(self.current_path_x))
            # rospy.loginfo("Current path length: %d", len(self.current_path_y))
            if len(self.current_path_x) < 2:
                # rospy.loginfo(self.current_path_x)
                rospy.loginfo("Waiting for waypoints...")
                self.rate.sleep()
                continue

            # Vehicle is at origin in global frame
            curr_x, curr_y = self.current_pose.position.x, self.current_pose.position.y

            # Calculate distances to all waypoints
            for i in range(self.wp_size):
                self.dist_arr[i] = self.dist(
                    (self.current_path_x[i], self.current_path_y[i]), 
                    (curr_x, curr_y)
                )
            # rospy.loginfo("Distances: %s", self.dist_arr)
            # Find points within look-ahead distance
            self.look_ahead = self.dist_arr.min()
            rospy.loginfo("Look-ahead distance: %f", self.look_ahead)
            goal_arr = np.where(
                (self.dist_arr < self.look_ahead + 0.5)
            )[0]
            # rospy.loginfo("Goal points: %s", goal_arr)
            # Find goal point starting from last goal
            start_idx = self.last_goal_index
            self.goal = start_idx

            # Publish used waypoints
            used_points = []
            for idx in goal_arr:
                used_points.append([self.current_path_x[idx], -self.current_path_y[idx]])
            self.publish_point_markers(self.kevin_msg, used_points, self.used_waypoints_pub)

            for idx in goal_arr:
                # if idx >= start_idx:
                v1 = [self.current_path_x[idx], self.current_path_y[idx]]
                v2 = [self.current_pose.position.x, self.current_pose.position.y]
                temp_angle = self.find_angle(v1, v2)
                if abs(temp_angle) < np.pi/2:
                    self.goal = idx
                    self.last_goal_index = idx
                    break

            # print current position
            rospy.loginfo("Current position: %f, %f", curr_x, curr_y)

            # print current goal position
            rospy.loginfo("Current goal: %d", self.goal)
            if self.goal == self.wp_size - 1:
                rospy.loginfo("Current goal position: %f, %f", self.current_path_x[self.goal], self.current_path_y[self.goal])
            
            # transform quaternion to euler angles
            x = self.current_pose.orientation.x
            y = self.current_pose.orientation.y
            z = self.current_pose.orientation.z
            w = self.current_pose.orientation.w
            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            curr_yaw = math.degrees(math.atan2(t0, t1))
            rospy.loginfo("Current yaw: %f", curr_yaw)

            # Calculate steering control
            L = self.dist_arr[self.goal]
            alpha = math.atan2(self.current_path_y[self.goal] - curr_y, self.current_path_x[self.goal] - curr_x) - curr_yaw
            rospy.loginfo("dx: %f, dy: %f", self.current_path_x[self.goal] - curr_x, self.current_path_y[self.goal] - curr_y)
            rospy.loginfo("Alpha: %f", alpha)
            # Pure pursuit control law
            k = 0.41  # Gain
            angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L)
            angle = angle_i * 2

            # Convert to steering angle
            f_delta = round(np.clip(angle, -0.61, 0.61), 3)
            f_delta_deg = np.degrees(f_delta)
            rospy.loginfo("Front wheel angle: %f", f_delta_deg)
            steering_angle = - self.front2steer(f_delta_deg)

            # if self.gem_enable:
                # Speed control
            current_time = rospy.get_time()
            filt_vel = self.speed_filter.get_data(self.speed)
            output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)
            output_accel = np.clip(output_accel, 0.2, self.max_accel)

            # Turn signal control based on steering angle
            if -30 <= f_delta_deg <= 30:
                self.turn_cmd.ui16_cmd = 1  # No signal
            elif f_delta_deg > 30:
                self.turn_cmd.ui16_cmd = 2  # Left
            else:
                self.turn_cmd.ui16_cmd = 0  # Right

            # Publish control commands
            self.accel_cmd.f64_cmd = output_accel
            self.steer_cmd.angular_position = np.radians(steering_angle)

            rospy.loginfo("Steering angle: %f", steering_angle)
            rospy.loginfo("Speed: %f", filt_vel)
            rospy.loginfo("Acceleration: %f", output_accel)

            self.accel_pub.publish(self.accel_cmd)
            self.steer_pub.publish(self.steer_cmd)
            self.turn_pub.publish(self.turn_cmd)

            rospy.loginfo("accel cmd: %f", self.accel_cmd.f64_cmd)
            rospy.loginfo("steer cmd: %f", self.steer_cmd.angular_position)
            rospy.loginfo("turn cmd: %d", self.turn_cmd.ui16_cmd)
            rospy.loginfo("_________________________")

            self.rate.sleep()

    def enable_pacmod(self):
        """Enable PACMod control system"""
        # Set forward gear
        self.gear_cmd.ui16_cmd = 3
        
        # Enable brake control
        self.brake_cmd.enable = True
        self.brake_cmd.clear = False
        self.brake_cmd.ignore = False
        self.brake_cmd.f64_cmd = 0.0

        # Enable acceleration control
        self.accel_cmd.enable = True
        self.accel_cmd.clear = False
        self.accel_cmd.ignore = False
        self.accel_cmd.f64_cmd = 0.0

        # Publish initial commands
        self.gear_pub.publish(self.gear_cmd)
        self.turn_pub.publish(self.turn_cmd)
        self.brake_pub.publish(self.brake_cmd)
        self.accel_pub.publish(self.accel_cmd)

        # Print control cmd
        rospy.loginfo("gear_cmd: %d", self.gear_cmd.ui16_cmd)
        rospy.loginfo("brake_cmd: %f", self.brake_cmd.f64_cmd)
        rospy.loginfo("accel_cmd: %f", self.accel_cmd.f64_cmd)


        self.gem_enable = True

def pure_pursuit():
    """Initialize and start the pure pursuit node"""
    rospy.init_node('lidar_pp_node', anonymous=True)
    pp = PurePursuit()
    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    pure_pursuit()
