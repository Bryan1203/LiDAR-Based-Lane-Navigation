import rospy
from nav_msgs.msg import Odometry

# 815 - 1081

def callback(msg: Odometry):
    print("Odometry Sequence:")
    print(f"  seq: {msg.child_frame_id}")
    # print(f"  seq: {msg.header.seq}")
    # print(f"  stamp: {msg.header.stamp}")
    # print(f"  frame_id: {msg.header.frame_id}")

rospy.init_node('display_odometry_header')
sub = rospy.Subscriber('/kiss/odometry', Odometry, callback)
rospy.spin()