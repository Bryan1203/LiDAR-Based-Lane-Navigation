import rospy
from sensor_msgs.msg import PointCloud2

# 1320 - 1944

def callback(msg: PointCloud2):
    print("PointCloud2 Sequence:")
    print(f"  seq: {msg.header.seq}")
    # print(f"  stamp: {msg.header.stamp}")
    # print(f"  frame_id: {msg.header.frame_id}")

rospy.init_node('display_pointcloud2_header')
sub = rospy.Subscriber('/ouster/points', PointCloud2, callback)
rospy.spin()