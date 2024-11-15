#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg

def point_cloud_callback(msg):
    # Desired height threshold
    height_threshold = -1.0  # Adjust this threshold as needed
    distance_threshold = 30

    # List to store filtered points
    filtered_points = []
    total_points = 0
    # Read points from the original PointCloud2 message
    for point in pc2.read_points(msg, skip_nans=True):
        x, y, z, = point[0],point[1],point[2]
        total_points+=1
        # Filter out points above the height threshold
        if z <= height_threshold and (x**2+y**2+z**2)**(1/2) <= distance_threshold and abs(y)<=10 and abs(x)<=20:
            filtered_points.append(point)
    # Create a new PointCloud2 message for the filtered data
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id
    # Use the same fields as the original message
    # fields = [msg.fields[i] for i in [0,1,2,7]]
    
    print(msg.fields)
    
    # Convert the filtered points to a PointCloud2 message
    filtered_msg = pc2.create_cloud(header, msg.fields, filtered_points)

    # Publish or store the filtered PointCloud2 message
    filtered_pub.publish(filtered_msg)
    print(f"{len(filtered_points)}/{total_points}")

def main():
    # Initialize the ROS node
    rospy.init_node('filtered_point_cloud_node', anonymous=True)
    
    # Subscribe to the 'ouster/points' topic
    rospy.Subscriber('ouster/points', PointCloud2, point_cloud_callback)
    # Publisher for the filtered PointCloud2 message
    global filtered_pub
    filtered_pub = rospy.Publisher('ouster/points_filtered', PointCloud2, queue_size=10)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
