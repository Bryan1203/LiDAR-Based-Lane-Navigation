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
        self.look_ahead = 4  # Look-ahead distance
        self.wheelbase = 2.57  # Vehicle wheelbase

        # Path storage and threading lock
        self.path_lock = threading.Lock()
        self.current_path_x = []  # Path points in vehicle's forward direction
        self.current_path_y = []  # Path points in vehicle's right direction
        self.wp_size = 0
        self.dist_arr = np.array([])
        self.last_goal_index = 0
        
        # Subscribe to topics
        self.waypoints_sub = rospy.Subscriber("/waypoints", Marker, self.waypoints_callback)
        self.speed_sub = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)
        
        # Vehicle state
        self.speed = 0.0
        self.gem_enable = False
        self.pacmod_enable = False
        
        # Speed control parameters
        self.desired_speed = 1.5
        self.max_accel = 0.5
        self.pid_speed = PID(0.5, 0.0, 0.1, wg=20)
        self.speed_filter = OnlineFilter(1.2, 30, 4)

        # Initialize PACMod control
        self.setup_pacmod_control()

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
        if msg.type != msg.POINTS:
            return
            
        with self.path_lock:
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

    def speed_callback(self, msg):
        """Update current vehicle speed"""
        self.speed = round(msg.vehicle_speed, 3)

    def enable_callback(self, msg):
        """Update PACMod enable status"""
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

    def start_pp(self):
        """Main pure pursuit control loop"""
        while not rospy.is_shutdown():
            # Initialize PACMod if not enabled
            if not self.gem_enable and self.pacmod_enable:
                self.enable_pacmod()
                
            with self.path_lock:
                if len(self.current_path_x) < 2:
                    rospy.loginfo("Waiting for waypoints...")
                    self.rate.sleep()
                    continue

                # Vehicle is at origin in local frame
                curr_x, curr_y = 0, 0

                # Calculate distances to all waypoints
                for i in range(self.wp_size):
                    self.dist_arr[i] = self.dist(
                        (self.current_path_x[i], self.current_path_y[i]), 
                        (curr_x, curr_y)
                    )

                # Find points within look-ahead distance
                goal_arr = np.where(
                    (self.dist_arr < self.look_ahead + 0.3) & 
                    (self.dist_arr > self.look_ahead - 0.3)
                )[0]

                # Find goal point starting from last goal
                start_idx = self.last_goal_index
                self.goal = start_idx

                for idx in goal_arr:
                    if idx >= start_idx:
                        v1 = [self.current_path_x[idx], self.current_path_y[idx]]
                        v2 = [1, 0]  # Vehicle's forward direction
                        temp_angle = self.find_angle(v1, v2)
                        if abs(temp_angle) < np.pi/2:
                            self.goal = idx
                            self.last_goal_index = idx
                            break

                # Calculate steering control
                L = self.dist_arr[self.goal]
                alpha = math.atan2(self.current_path_y[self.goal], 
                                 self.current_path_x[self.goal])

                # Pure pursuit control law
                k = 0.41  # Gain
                angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L)
                angle = angle_i * 2

                # Convert to steering angle
                f_delta = round(np.clip(angle, -0.61, 0.61), 3)
                f_delta_deg = np.degrees(f_delta)
                steering_angle = self.front2steer(f_delta_deg)

                if self.gem_enable:
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
                    self.accel_pub.publish(self.accel_cmd)
                    self.steer_pub.publish(self.steer_cmd)
                    self.turn_pub.publish(self.turn_cmd)

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