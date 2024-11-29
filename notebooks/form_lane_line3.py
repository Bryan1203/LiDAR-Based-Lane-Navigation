import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
import time

# Load the lane line labels and point cloud data
lane_labels = np.load(r"C:\Users\kkhua\Desktop\ECE484\Final Project\slam06_laneline.npy")
point_cloud = np.load(r"C:\Users\kkhua\Desktop\ECE484\Final Project\slam06_mapping_global.npy")

# Get lane points from first 50 frames
num_frame = 10
all_lane_points = []
for frame in range(num_frame):
    frame_lane_indices = lane_labels[frame] == 1
    frame_points = point_cloud[frame].reshape(128*1024, 7)
    lane_points = frame_points[frame_lane_indices,:]
    all_lane_points.append(lane_points)

# Combine all lane points
lane_line_points = np.vstack(all_lane_points)

start_time = time.time()

# ---------------------------CLUSTERING with KD-Tree--------------------------------#
# Use only x,y coordinates for clustering
lane_xy = lane_line_points[:, :2]

# Build KD-tree
tree = cKDTree(lane_xy)

# Parameters for clustering
eps = 1.0  # Distance threshold
min_samples = 10  # Minimum points for a cluster

# Find all points within eps distance
indices = tree.query_ball_point(lane_xy, eps)

# Initialize labels
labels = np.full(len(lane_xy), -1)
cluster_id = 0

# Clustering algorithm similar to DBSCAN
for i in range(len(indices)):
    if labels[i] != -1:  # Skip if point already assigned
        continue
        
    neighbors = indices[i]
    if len(neighbors) < min_samples:  # Mark as noise if not enough neighbors
        continue
        
    # Start a new cluster
    labels[i] = cluster_id
    
    # Expand cluster
    seed_set = neighbors.copy()
    while seed_set:
        current_point = seed_set.pop()
        if labels[current_point] == -1:
            labels[current_point] = cluster_id
            current_neighbors = indices[current_point]
            if len(current_neighbors) >= min_samples:
                seed_set.extend([n for n in current_neighbors if labels[n] == -1])
    
    cluster_id += 1

cluster_time = time.time()
print(f'cluster time: {cluster_time-start_time}')
lane_line_points[:, 4] = labels  # Assign cluster IDs as labels

# Create a single plot for all clusters
plt.figure(figsize=(12, 8))

# Plot each cluster
colors = plt.cm.rainbow(np.linspace(0, 1, len(np.unique(labels))-1))  # -1 to exclude noise
color_idx = 0

for cluster_id in np.unique(labels):
    if cluster_id == -1:  # Skip noise points
        continue
        
    # Extract points for this cluster
    cluster_points = lane_line_points[lane_line_points[:, 4] == cluster_id]
    X_cluster = cluster_points[:, 0]
    Y_cluster = cluster_points[:, 1]
    
    # Fit a line using polyfit
    coeffs = np.polyfit(X_cluster, Y_cluster, 1)
    X_range = np.linspace(np.min(X_cluster), np.max(X_cluster), 100)
    Y_pred = np.polyval(coeffs, X_range)
    
    # Plot the points and fitted line
    plt.scatter(X_cluster, Y_cluster, color=colors[color_idx], label=f'Cluster {int(cluster_id)} Points')
    plt.plot(X_range, Y_pred, '--', color=colors[color_idx], label=f'Cluster {int(cluster_id)} Line')
    color_idx += 1

plt.title(f'Lane Lines Detection using KD-Tree Clustering ({num_frame} Frames)')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.legend()
plt.grid(True)
plt.show()

# # --------------- Publish the Lane Lines ---------------------#
# import rospy
# from sensor_msgs.msg import PointCloud2
# import sensor_msgs.point_cloud2 as pc2

# rospy.init_node("lane_line_kdtree_publisher")
# pub = rospy.Publisher("/kdtree_lane_lines", PointCloud2, queue_size=10)

# header = rospy.Header()
# header.frame_id = "map"

# # Convert fitted lines to PointCloud2
# while not rospy.is_shutdown():
#     all_points = []
#     for cluster_id in np.unique(labels):
#         if cluster_id == -1:
#             continue
#         cluster_points = lane_line_points[lane_line_points[:, 4] == cluster_id]
#         X_cluster = cluster_points[:, 0]
#         Y_cluster = cluster_points[:, 1]
#         coeffs = np.polyfit(X_cluster, Y_cluster, 1)
#         X_range = np.linspace(np.min(X_cluster), np.max(X_cluster), 100)
#         Y_pred = np.polyval(coeffs, X_range)
#         for x, y in zip(X_range, Y_pred):
#             all_points.append((x, y, 0.0))

#     # Create and publish PointCloud2
#     cloud_msg = pc2.create_cloud_xyz32(header, all_points)
#     pub.publish(cloud_msg)
#     rospy.sleep(0.1)