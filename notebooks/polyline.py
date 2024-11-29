import numpy as np
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
import time
from collections import Counter

# Load the lane line labels and point cloud data
lane_labels = np.load(r"C:\Users\kkhua\Desktop\ECE484\Final Project\slam06_laneline.npy")
point_cloud = np.load(r"C:\Users\kkhua\Desktop\ECE484\Final Project\slam06_mapping_global.npy")

# Get lane points from the first 50 frames
num_frame = 10
all_lane_points = []
for frame in range(num_frame):
    frame_lane_indices = lane_labels[frame * 10] == 1
    frame_points = point_cloud[frame * 10].reshape(128 * 1024, 7)
    lane_points = frame_points[frame_lane_indices, :]
    all_lane_points.append(lane_points)

# Combine all lane points
lane_line_points = np.vstack(all_lane_points)

start_time = time.time()

# ---------------------------CLUSTERING with KD-Tree--------------------------------#
# Use only x, y coordinates for clustering
lane_xy = lane_line_points[:, :2]

# Build KD-tree
tree = cKDTree(lane_xy)

# Parameters for clustering
eps = 0.5  # Distance threshold
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
print(f'Cluster time: {cluster_time - start_time}')
lane_line_points[:, 4] = labels  # Assign cluster IDs as labels

# Find the most common label
most_common_label = Counter(labels).most_common(1)[0][0]

# Apply polyline growing to the most common label
most_common_points = lane_line_points[lane_line_points[:, 4] == most_common_label]
most_common_xy = most_common_points[:, :2]

# # --------------- Publish the Lane Lines ---------------------#
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

rospy.init_node("lane_line_publisher")