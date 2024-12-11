#!/usr/bin/env python3

#================================================================
# File name: gem_gnss_pp_tracker_pid.py                                                                  
# Description: gnss waypoints tracker using pid and pure pursuit                                                                
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 08/02/2021                                                                
# Date last modified: 08/15/2022                                                          
# Version: 1.0                                                                   
# Usage: rosrun gem_gnss gem_gnss_pp_tracker.py                                                                      
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
# import alvinxy.alvinxy as axy # Import AlvinXY transformation module
import rospy
import tf

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

# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt


class PID(object):

    def __init__(self, kp, ki, kd, wg=None):

        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0

    def reset(self):
        self.iterm  = 0
        self.last_e = 0
        self.last_t = None

    def get_control(self, t, e, fwd=0):

        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0

        self.iterm += e * (t - self.last_t)
        # self.iterm = 0.0 

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de
        rospy.logwarn(f"Enter PID control {fwd + self.kp * e + self.ki * self.iterm + self.kd * de}")
        
        # return np.clip(fwd + self.kp * e + self.ki * self.iterm + self.kd * de,0, 0.3)
        # if fwd + self.kp * e + self.ki * self.iterm + self.kd * de < 0.33:
        #     return 0.3
        # return abs(fwd + self.kp * e + self.ki * self.iterm + self.kd * de)
        return np.clip(fwd + self.kp * e + self.ki * self.iterm + self.kd * de, 0, 0.55)


class OnlineFilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coefficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):
        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted


class PurePursuit(object):
    
    def __init__(self):

        self.rate       = rospy.Rate(10 )

        self.look_ahead = 8
        self.wheelbase  = 1.8 # meters
        self.offset     = 1.26 # meters

        self.last_goal_x = 0
        self.last_goal_y = 0
        self.path_points_x = []
        self.path_points_y = []
        # self.path_points_x = []
        # self.path_points_y = []
        self.wp_size = 0
        self.dist_arr = np.array([0])
        self.wpts_msg = Marker()
        self.waypoint_received = False
        self.waypoint_reached = True

        # self.gnss_sub   = rospy.Subscriber("/novatel/inspva", Inspva, self.inspva_callback)

        # self.gnss_sub   = rospy.Subscriber("/septentrio_gnss/navsatfix", NavSatFix, self.gnss_callback)
        # self.ins_sub    = rospy.Subscriber("/septentrio_gnss/insnavgeod", INSNavGeod, self.ins_callback)
        self.global_pose_sub = rospy.Subscriber("/waypoint/pose", Odometry, self.global_pose_callback, queue_size=1)
        self.waypoints_sub = rospy.Subscriber("/waypoint_marker", Marker, self.waypoints_callback, queue_size=1)

        self.lat        = 0.0
        self.lon        = 0.0
        self.heading    = 0.0

        self.enable_sub = rospy.Subscriber("/pacmod/as_tx/enable", Bool, self.enable_callback)

        self.speed_sub  = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        self.speed      = 0.0

        self.used_waypoints_pub = rospy.Publisher("/used_waypoints", Marker, queue_size=1)
        self.pose_pub = rospy.Publisher('/current_pose', PoseStamped, queue_size=1)

        # self.olat       = 40.092855    
        # self.olon       = -88.235981 

        # read waypoints into the system 
        self.goal       = 0            
        # self.read_waypoints() 

        self.desired_speed = 0.6  # m/s, reference speed
        self.max_accel     = 0.48 # % of acceleration
        self.pid_speed     = PID(0.2, 0.2, 0.0, wg=20)
        self.speed_filter  = OnlineFilter(1.2, 30, 4)


        # -------------------- PACMod setup --------------------
        self.overspeed = False
        self.gem_enable    = False
        self.pacmod_enable = False

        # GEM vehicle enable, publish once
        self.enable_pub = rospy.Publisher('/pacmod/as_rx/enable', Bool, queue_size=1)
        self.enable_cmd = Bool()
        self.enable_cmd.data = False

        # GEM vehicle gear control, neutral, forward and reverse, publish once
        self.gear_pub = rospy.Publisher('/pacmod/as_rx/shift_cmd', PacmodCmd, queue_size=1)
        self.gear_cmd = PacmodCmd()
        self.gear_cmd.ui16_cmd = 2 # SHIFT_NEUTRAL

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

        # GEM vechile forward motion control
        self.accel_pub = rospy.Publisher('/pacmod/as_rx/accel_cmd', PacmodCmd, queue_size=1)
        self.accel_cmd = PacmodCmd()
        self.accel_cmd.enable = False
        self.accel_cmd.clear  = True
        self.accel_cmd.ignore = True

        # GEM vechile turn signal control
        self.turn_pub = rospy.Publisher('/pacmod/as_rx/turn_cmd', PacmodCmd, queue_size=1)
        self.turn_cmd = PacmodCmd()
        self.turn_cmd.ui16_cmd = 1 # None

        # GEM vechile steering wheel control
        self.steer_pub = rospy.Publisher('/pacmod/as_rx/steer_cmd', PositionWithSpeed, queue_size=1)
        self.steer_cmd = PositionWithSpeed()
        self.steer_cmd.angular_position = 0.0 # radians, -: clockwise, +: counter-clockwise
        self.steer_cmd.angular_velocity_limit = 3.5 # radians/second

    def ins_callback(self, msg):
        rospy.loginfo("INS info: %s", msg)
        # self.heading = round(msg.heading, 6)
        # self.pose_pub.publish(msg)

    # def gnss_callback(self, msg):
    #     self.lat = round(msg.latitude, 6)
    #     self.lon = round(msg.longitude, 6)

    def global_pose_callback(self, msg):
        # calculate quaternion to euler angle
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.heading = round(math.degrees(tf.transformations.euler_from_quaternion(quaternion)[2]), 6)
        self.lat = round(msg.pose.pose.position.x, 6)
        self.lon = round(msg.pose.pose.position.y, 6)
        # if len(self.path_points_x) > 0 and len(self.path_points_y) > 0:

        #     distance = self.dist((self.path_points_x[self.goal], self.path_points_y[self.goal]), (msg.pose.pose.position.x, msg.pose.pose.position.y))
        #     rospy.logwarn(f"Current Distant from Next Waypoint {distance}")
        #     if distance < 0.5:
        #         rospy.logwarn("REACHED WAYPOINTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT")
        #         self.waypoint_reached = True

    def waypoints_callback(self, msg):
        # if self.waypoint_reached:
        rospy.logwarn("New Waypoints Received")
        self.wpts_msg = msg
        self.path_points_x = []
        self.path_points_y = []
        for i in range(len(msg.points)):
            self.path_points_x.append(msg.points[i].x)
            self.path_points_y.append(msg.points[i].y)
        self.wp_size = len(self.path_points_x)
        self.dist_arr = np.zeros(self.wp_size)
        # rospy.loginfo("way points info: %s", self.wp_size)
        # rospy.loginfo("way points x: %s", self.path_points_x)
        self.waypoint_reached = False
        

    def speed_callback(self, msg):
        
        # rospy.logwarn("Vehicle speed: %s", msg.vehicle_speed)
        
        self.speed = round(msg.vehicle_speed, 3) # forward velocity in m/s
        if self.speed > self.desired_speed + 0.3: 
            self.overspeed = True

    def enable_callback(self, msg):
        self.pacmod_enable = msg.data

    def heading_to_yaw(self, heading_curr):
        if (heading_curr >= 270 and heading_curr < 360):
            yaw_curr = np.radians(450 - heading_curr)
        else:
            yaw_curr = np.radians(90 - heading_curr)
        return yaw_curr

    def front2steer(self, f_angle):
        if(f_angle > 35):
            f_angle = 35
        if (f_angle < -35):
            f_angle = -35
        if (f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0
        return steer_angle

    # def read_waypoints(self):
    #     # read recorded GPS lat, lon, heading
    #     dirname  = os.path.dirname(__file__)
    #     filename = os.path.join(dirname, '../waypoints/xyhead_demo_pp.csv')
    #     with open(filename) as f:
    #         path_points = [tuple(line) for line in csv.reader(f)]
    #     # x towards East and y towards North
    #     self.path_points_x   = [float(point[0]) for point in path_points] # longitude
    #     self.path_points_y   = [float(point[1]) for point in path_points] # latitude
    #     self.path_points_heading = [float(point[2]) for point in path_points] # heading
    #     self.wp_size             = len(self.path_points_x)
    #     self.dist_arr            = np.zeros(self.wp_size)



    # def wps_to_local_xy(self, lon_wp, lat_wp):
    #     # convert GNSS waypoints into local fixed frame reprented in x and y
    #     lon_wp_x, lat_wp_y = axy.ll2xy(lat_wp, lon_wp, self.olat, self.olon)
    #     return lon_wp_x, lat_wp_y   

    def get_gem_state(self):

        # vehicle gnss heading (yaw) in degrees
        # vehicle x, y position in fixed local frame, in meters
        # reference point is located at the center of GNSS antennas
        # local_x_curr, local_y_curr = self.wps_to_local_xy(self.lon, self.lat)
        local_x_curr = self.lat
        local_y_curr = self.lon

        # heading to yaw (degrees to radians)
        # heading is calculated from two GNSS antennas
        # curr_yaw = self.heading_to_yaw(self.heading) 
        curr_yaw = np.radians(self.heading)

        # reference point is located at the center of rear axle
        # curr_x = local_x_curr - self.offset * np.cos(curr_yaw)
        # curr_y = local_y_curr - self.offset * np.sin(curr_yaw)
        curr_x = local_x_curr
        curr_y = local_y_curr

        return round(curr_x, 3), round(curr_y, 3), round(curr_yaw, 4)

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        # [-pi, pi]
        return np.arctan2(sinang, cosang)

    # computes the Euclidean distance between two 2D points
    def dist(self, p1, p2):
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)
    
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
        
        while not rospy.is_shutdown():

            if (self.gem_enable == False):

                if(self.pacmod_enable == True):

                    # ---------- enable PACMod ----------

                    # enable forward gear
                    self.gear_cmd.ui16_cmd = 3

                    # enable brake
                    self.brake_cmd.enable  = True
                    self.brake_cmd.clear   = False
                    self.brake_cmd.ignore  = False
                    self.brake_cmd.f64_cmd = 0.0

                    # enable gas 
                    self.accel_cmd.enable  = True
                    self.accel_cmd.clear   = False
                    self.accel_cmd.ignore  = False
                    self.accel_cmd.f64_cmd = 0.0

                    self.gear_pub.publish(self.gear_cmd)
                    print("Foward Engaged!")

                    self.turn_pub.publish(self.turn_cmd)
                    print("Turn Signal Ready!")
                    
                    self.brake_pub.publish(self.brake_cmd)
                    print("Brake Engaged!")

                    self.accel_pub.publish(self.accel_cmd)
                    print("Gas Engaged!")

                    self.gem_enable = True

            # rospy.loginfo("GEM enable status: %s", self.gem_enable)
            # rospy.loginfo("path points size: %s", self.wp_size)

            curr_x, curr_y, curr_yaw = self.get_gem_state()
            rospy.loginfo("Current x: %f, y: %f, yaw: %f", curr_x, curr_y, curr_yaw)
            if self.path_points_x == []:
                rospy.loginfo("No waypoints received")
                self.waypoint_received = False
            else:
                self.waypoint_received = True

            if self.waypoint_received:
                # finding the distance of each way point from the current position
                for i in range(len(self.path_points_x)):
                    self.dist_arr[i] = self.dist((self.path_points_x[i], self.path_points_y[i]), (curr_x, curr_y))
                
                
                self.look_ahead = self.dist_arr.min()
                # finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)
                goal_arr = np.where( ((self.dist_arr > self.look_ahead + 0.5) & (self.dist_arr < self.look_ahead + 1)))[0]

                if len(goal_arr) == 0:
                    rospy.loginfo("No waypoints ahead")
                    goal_arr = np.where(self.dist_arr == self.look_ahead)[0]

                # rospy.loginfo("path points x: %s", self.path_points_x)
                # rospy.loginfo("goal array: %s", goal_arr)   
                # Publish used waypoints

                used_points = []
                for idx in goal_arr:
                    used_points.append([self.path_points_x[idx], self.path_points_y[idx]])
                self.publish_point_markers(self.wpts_msg, used_points, self.used_waypoints_pub)

                # finding the goal point which is the last in the set of points less than the lookahead distance
                for idx in goal_arr:
                    v1 = [self.path_points_x[idx]-curr_x , self.path_points_y[idx]-curr_y]
                    v2 = [np.cos(curr_yaw), np.sin(curr_yaw)]
                    temp_angle = self.find_angle(v1,v2)
                    # find correct look-ahead point by using heading information
                    if abs(temp_angle) < np.pi/2:
                        self.goal = idx
                        break

                self.last_goal_x = self.path_points_x[self.goal]
                self.last_goal_y = self.path_points_y[self.goal]

                # finding the distance between the goal point and the vehicle
                # true look-ahead distance between a waypoint and current position
                L = self.dist_arr[self.goal]

                # find the curvature and the angle 
                rospy.loginfo("Goal index: %s", self.goal)
                
                rospy.loginfo("path length: %s", self.wp_size)
                # rospy.loginfo(self.path_points_heading)
                # alpha = self.heading_to_yaw(self.path_points_heading[self.goal]) - curr_yaw
                try:
                    desired_yaw = math.atan2(self.path_points_y[self.goal] - curr_y, self.path_points_x[self.goal] - curr_x)
                    rospy.loginfo("desired x: %f, y: %f, yaw: %f", self.path_points_x[self.goal], self.path_points_y[self.goal], desired_yaw)
                # desired_yaw = math.atan2(self.path_points_y[self.goal] - curr_y, self.path_points_x[self.goal] - curr_x)
                # rospy.loginfo("desired x: %f, y: %f, yaw: %f", self.path_points_x[self.goal], self.path_points_y[self.goal], desired_yaw)
                except:
                    rospy.logwarn("New Index Out of Range")
            else:
                rospy.loginfo("No waypoints received, using last goal point")
                L = self.dist((self.last_goal_x, self.last_goal_y), (curr_x, curr_y))
                desired_yaw = math.atan2(self.last_goal_y - curr_y, self.last_goal_x - curr_x)
            
            # calculate the angle between the vehicle heading and the line connecting the vehicle and the goal point
            alpha =  desired_yaw - curr_yaw

            
            # ----------------- tuning this part as needed -----------------
            k       = 0.41 
            angle_i = math.atan((k * 2 * self.wheelbase * math.sin(alpha)) / L) 
            angle   = angle_i*2
            # ----------------- tuning this part as needed -----------------

            f_delta = round(np.clip(angle, -0.61, 0.61), 3)

            f_delta_deg = np.degrees(f_delta)
            rospy.loginfo("Front wheel angle: %f", f_delta_deg)
            # steering_angle in degrees
            steering_angle = self.front2steer(f_delta_deg)

            if(self.gem_enable == True):
                print("Current index: " + str(self.goal))
                print("Forward velocity: " + str(self.speed))
                ct_error = round(np.sin(alpha) * L, 3)
                print("Crosstrack Error: " + str(ct_error))
                print("Front steering angle: " + str(np.degrees(f_delta)) + " degrees")
                print("Steering wheel angle: " + str(steering_angle) + " degrees" )
                print("\n")

            current_time = rospy.get_time()
            filt_vel     = self.speed_filter.get_data(self.speed)
            output_accel = self.pid_speed.get_control(current_time, self.desired_speed - filt_vel)

            if output_accel > self.max_accel:
                output_accel = self.max_accel

            if self.overspeed:
                self.overspeed = False
                rospy.logwarn("Overspeed")
                output_accel = 0.1

            if output_accel < 0.1:
                rospy.logwarn("Under Acceleration")
                output_accel = 0.1

            if (f_delta_deg <= 30 and f_delta_deg >= -30):
                self.turn_cmd.ui16_cmd = 1
            elif(f_delta_deg > 30):
                self.turn_cmd.ui16_cmd = 2 # turn left
            else:
                self.turn_cmd.ui16_cmd = 0 # turn right

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


def pure_pursuit():

    rospy.init_node('lidar_pp_node', anonymous=True)
    pp = PurePursuit()

    try:
        pp.start_pp()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()


