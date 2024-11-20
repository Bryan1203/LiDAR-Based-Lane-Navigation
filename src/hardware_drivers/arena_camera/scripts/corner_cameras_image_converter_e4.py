#!/usr/bin/env python3

#================================================================
# File name: corner_cameras_image_converter.py                                                                  
# Description: convert raw images of corner cameras to rgb images                                                         
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 01/21/2024                                                               
# Date last modified: 01/21/2024                                                          
# Version: 0.1                                                                    
# Usage: rosrun arena_camera corner_cameras_image_converter.py                                                                  
# Python version: 3.8                                                             
#================================================================

from __future__ import print_function

import sys
import copy
import time
import rospy
import rospkg

import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

import message_filters
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class ImageConverter:

    def __init__(self):

        rospy.init_node("corner_cameras_image_converter")
        
        self.bridge = CvBridge()
        self.fl_image_sub = message_filters.Subscriber("/camera_fl/arena_camera_node/image_raw", Image)
        self.fr_image_sub = message_filters.Subscriber("/camera_fr/arena_camera_node/image_raw", Image)
        self.rl_image_sub = message_filters.Subscriber("/camera_rl/arena_camera_node/image_raw", Image)
        self.rr_image_sub = message_filters.Subscriber("/camera_rr/arena_camera_node/image_raw", Image)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.fl_image_sub, self.fr_image_sub, 
                                                               self.rl_image_sub, self.rr_image_sub], 
                                                               10, 0.1, allow_headerless=True)

        self.ts.registerCallback(self.image_callback)

        self.corner_image_pub = rospy.Publisher("/camera_corners/arena_camera_node/image_raw", Image, queue_size=10)

    def image_callback(self, fl_image, fr_image, rl_image, rr_image):

        try:
            fl_frame = self.bridge.imgmsg_to_cv2(fl_image, "passthrough")
            fr_frame = self.bridge.imgmsg_to_cv2(fr_image, "passthrough")
            rl_frame = self.bridge.imgmsg_to_cv2(rl_image, "passthrough")
            rr_frame = self.bridge.imgmsg_to_cv2(rr_image, "passthrough")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        f_frame = np.concatenate((fl_frame, fr_frame), axis=1) # combine horizontal
        r_frame = np.concatenate((rl_frame, rr_frame), axis=1) # combine horizontal
        frame = np.concatenate((f_frame, r_frame), axis=0)     # combine vertical
                
        try:
            self.corner_image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "rgb8"))
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

def main(args):
    try:
        ImageConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS node.")

if __name__ == '__main__':
    main(sys.argv)
