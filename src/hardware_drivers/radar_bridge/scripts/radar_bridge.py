#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

class RadarBridge(object):

    def __init__(self):
        self.radar_sub = rospy.Subscriber("/smart_radar/targets_0", PointCloud2, self.radar_callback)

    def radar_callback(self, msg):
        pass
    
    def start(self):
        while not rospy.is_shutdown():
            pass
  
def main():
    rospy.init_node('ros2_to_ros1_radar', anonymous=True)
    radar_ob = RadarBridge()
    try:
        radar_ob.start()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()