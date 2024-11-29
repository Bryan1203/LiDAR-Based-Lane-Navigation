import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
import matplotlib.pyplot as plt

# Load the lane line labels and point cloud data
lane_labels = np.load(r"D:\484_final_project\processed_point_cloud\slam06_laneline.npy")
point_cloud = np.load(r"D:\484_final_project\processed_point_cloud\slam06_mapping_global.npy")

# Get lane point indices from frame 0 
frame0_lane_indices = lane_labels[0] == 1

# Extract lane points from point cloud using frame 0 indices
frame0_points = point_cloud[0].reshape(-1,point_cloud.shape[1])
lane_line_points = frame0_points[frame0_lane_indices]


# ---------------------------CLUSTERING and RANSAC--------------------------------#
# Cluster the points first
lane_xy = lane_line_points[:, :2]  # Use x and y for clustering
dbscan = DBSCAN(eps=1.0, min_samples=10).fit(lane_xy)
lane_line_points[:, 4] = dbscan.labels_  # Assign cluster IDs as labels

# Fit RANSAC to each cluster
fitted_lines = {}
for cluster_id in np.unique(dbscan.labels_):
    if cluster_id == -1:  # Skip noise points
        continue

    # Extract points for this cluster
    cluster_points = lane_line_points[lane_line_points[:, 4] == cluster_id]
    X_cluster = cluster_points[:, 0].reshape(-1, 1)
    Y_cluster = cluster_points[:, 1]

    # Fit RANSAC to the cluster
    ransac = make_pipeline(PolynomialFeatures(degree=1), RANSACRegressor())
    ransac.fit(X_cluster, Y_cluster)

    # Predict the line for visualization or further use
    X_range = np.linspace(np.min(X_cluster), np.max(X_cluster), 100).reshape(-1, 1)
    Y_pred = ransac.predict(X_range)
    fitted_lines[cluster_id] = (X_range, Y_pred)
    # Plot the points and fitted line
    plt.scatter(X_cluster, Y_cluster, color='blue', label='Lane Points')
    plt.plot(X_range, Y_pred, color='red', label='RANSAC Lane Line')
    plt.legend()
    plt.show()

# # --------------- Publish the Lane Lines ---------------------#
# import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2

# rospy.init_node("lane_line_ransac_publisher")
# pub = rospy.Publisher("/ransac_lane_lines", PointCloud2, queue_size=10)

# header = rospy.Header()
# header.frame_id = "map"

# # Convert fitted lines to PointCloud2
# while not rospy.is_shutdown():
#     all_points = []
#     for cluster_id, (X_range, Y_pred) in fitted_lines.items():
#         for x, y in zip(X_range.flatten(), Y_pred):
#             all_points.append((x, y, 0.0))  # z = 0 for simplicity

#     # Create and publish PointCloud2
#     cloud_msg = pc2.create_cloud_xyz32(header, all_points)
#     pub.publish(cloud_msg)
#     rospy.sleep(0.1)
