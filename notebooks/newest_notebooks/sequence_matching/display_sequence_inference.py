import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# 1320 - 1944

def callback(msg: PointCloud2):
    first_point = next(pc2.read_points(msg, skip_nans=True))
    print("PointCloud2 Sequence:")
    print(f"  seq: {first_point[6]}")

rospy.init_node('display_pointcloud2_header')
sub = rospy.Subscriber('ouster/points_inference', PointCloud2, callback)
rospy.spin()