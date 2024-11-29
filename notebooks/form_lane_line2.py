import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.linear_model import RANSACRegressor
from sklearn.preprocessing import PolynomialFeatures
from sklearn.pipeline import make_pipeline
import matplotlib.pyplot as plt
import time
import pptk

start_time = time.time()
# Load the lane line labels and point cloud data
lane_labels = np.load(r"C:\Users\kkhua\Desktop\ECE484\Final Project\slam06_laneline.npy")
point_cloud = np.load(r"C:\Users\kkhua\Desktop\ECE484\Final Project\slam06_mapping_global.npy")

# Get lane point indices from frame 0 
frame0_lane_indices = lane_labels[0] == 1

# Extract lane points from point cloud using frame 0 indices
frame0_points = point_cloud[0].reshape(128*1024, 7)
lane_line_points = frame0_points[frame0_lane_indices,:]
plt.figure(figsize=(10, 8))

# ---------------------------CLUSTERING with KD-Tree--------------------------------#
# Use x and y coordinates for clustering
lane_xy = lane_line_points[:, :2]

# Build KD-tree
kdtree = pptk.kdtree._build(lane_xy, maxleafsize=10)

# Find neighbors within radius 2.0
clusters = []
used_points = set()

for i in range(len(lane_xy)):
   if i in used_points:
       continue
       
   # Query points within larger radius (2.0 instead of 1.0)
   neighbors = pptk.kdtree._query(kdtree, lane_xy[i:i+1], dmax=2.0)
   if len(neighbors[0]) >= 5:  # Increased min samples to 5
       cluster = set(neighbors[0])
       # Expand cluster by checking neighbors of neighbors
       cluster_expanded = cluster.copy()
       for point_idx in cluster:
           if point_idx not in used_points:
               new_neighbors = pptk.kdtree._query(kdtree, lane_xy[point_idx:point_idx+1], dmax=2.0)
               cluster_expanded.update(new_neighbors[0])
       
       used_points.update(cluster_expanded)
       clusters.append(list(cluster_expanded))

# Assign cluster labels
labels = np.full(len(lane_xy), -1)
for i, cluster in enumerate(clusters):
   labels[cluster] = i

lane_line_points = np.column_stack((lane_line_points, labels))

cluster_time = time.time()
print(f"Time taken for clustering: {cluster_time - start_time} seconds")

# Fit RANSAC to each cluster
fitted_lines = {}

for cluster_id in np.unique(labels):
   if cluster_id == -1:  # Skip noise points
       continue

   # Extract points for this cluster
   cluster_points = lane_line_points[lane_line_points[:, -1] == cluster_id]
   X_cluster = cluster_points[:, 0].reshape(-1, 1)
   Y_cluster = cluster_points[:, 1]

   # Fit RANSAC to the cluster
   ransac = make_pipeline(PolynomialFeatures(degree=1), RANSACRegressor())
   ransac.fit(X_cluster, Y_cluster)

   # Calculate appropriate number of points based on map size
   map_length = np.max(X_cluster) - np.min(X_cluster)
   points_per_meter = 10  # Adjust this value based on desired resolution
   num_points = int(map_length * points_per_meter)
   
   X_range = np.linspace(np.min(X_cluster), np.max(X_cluster), num_points).reshape(-1, 1)
   Y_pred = ransac.predict(X_range)
   fitted_lines[cluster_id] = (X_range, Y_pred)
   
   # Plot the points and fitted line for this cluster
   plt.scatter(X_cluster, Y_cluster, label=f'Lane Points Cluster {cluster_id}')
   plt.plot(X_range, Y_pred, '--', label=f'RANSAC Line Cluster {cluster_id}')

end_time = time.time()
ransac_time = end_time - cluster_time
print(f"Time taken for RANSAC: {ransac_time} seconds")
plt.xlabel('X')
plt.ylabel('Y') 
plt.title('Lane Lines Detection')
plt.legend()
plt.grid(True)
plt.show()
