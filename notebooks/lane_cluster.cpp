#include <pptk/kdtree.h>
#include <vector>
#include <memory>
#include <chrono>
#include <iostream>
#include <Eigen/Dense>
#include <cnpy.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <cmath>

class LaneLineClusterer {
private:
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_;
    float eps_;  // clustering radius
    int min_points_;  // minimum points for a cluster
    
public:
    LaneLineClusterer(float eps = 1.0, int min_points = 2) 
        : eps_(eps), min_points_(min_points) {}

    std::vector<std::vector<int>> clusterPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        // Initialize KD-tree
        kdtree_.setInputCloud(cloud);
        
        std::vector<bool> processed(cloud->points.size(), false);
        std::vector<std::vector<int>> clusters;
        
        // Process each point
        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (processed[i]) continue;
            
            std::vector<int> cluster;
            expandCluster(cloud, i, processed, cluster);
            
            if (cluster.size() >= min_points_) {
                clusters.push_back(cluster);
            }
        }
        
        return clusters;
    }

private:
    void expandCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      int point_idx,
                      std::vector<bool>& processed,
                      std::vector<int>& cluster) {
        
        std::vector<int> neighbor_indices;
        std::vector<float> distances;
        
        // Find neighbors
        kdtree_.radiusSearch(cloud->points[point_idx], eps_, 
                            neighbor_indices, distances);
        
        processed[point_idx] = true;
        cluster.push_back(point_idx);
        
        // Process neighbors
        for (size_t i = 0; i < neighbor_indices.size(); ++i) {
            int neighbor_idx = neighbor_indices[i];
            
            if (!processed[neighbor_idx]) {
                processed[neighbor_idx] = true;
                
                std::vector<int> neighbor_neighbors;
                std::vector<float> neighbor_distances;
                
                kdtree_.radiusSearch(cloud->points[neighbor_idx], eps_,
                                   neighbor_neighbors, neighbor_distances);
                
                if (neighbor_neighbors.size() >= min_points_) {
                    for (int idx : neighbor_neighbors) {
                        if (std::find(neighbor_indices.begin(), 
                                    neighbor_indices.end(), idx) 
                            == neighbor_indices.end()) {
                            neighbor_indices.push_back(idx);
                        }
                    }
                }
                
                if (std::find(cluster.begin(), cluster.end(), neighbor_idx) 
                    == cluster.end()) {
                    cluster.push_back(neighbor_idx);
                }
            }
        }
    }
};

class RANSACRegressor {
private:
    int n_iterations_;
    double residual_threshold_;
    Eigen::VectorXd coefficients_;

public:
    RANSACRegressor(int n_iterations = 100, double residual_threshold = 0.1)
        : n_iterations_(n_iterations), residual_threshold_(residual_threshold) {}

    void fit(const std::vector<double>& X, const std::vector<double>& y) {
        int n_samples = X.size();
        int best_inliers_count = 0;
        
        for (int iter = 0; iter < n_iterations_; ++iter) {
            // Randomly select two points
            int idx1 = rand() % n_samples;
            int idx2 = rand() % n_samples;
            
            // Fit line to these points
            double x1 = X[idx1], y1 = y[idx1];
            double x2 = X[idx2], y2 = y[idx2];
            
            double slope = (y2 - y1) / (x2 - x1);
            double intercept = y1 - slope * x1;
            
            // Count inliers
            int inliers_count = 0;
            std::vector<int> inlier_indices;
            
            for (int i = 0; i < n_samples; ++i) {
                double pred = slope * X[i] + intercept;
                double residual = std::abs(y[i] - pred);
                
                if (residual < residual_threshold_) {
                    inliers_count++;
                    inlier_indices.push_back(i);
                }
            }
            
            if (inliers_count > best_inliers_count) {
                best_inliers_count = inliers_count;
                
                // Refit using all inliers
                if (inlier_indices.size() > 1) {
                    Eigen::MatrixXd X_inliers(inlier_indices.size(), 2);
                    Eigen::VectorXd y_inliers(inlier_indices.size());
                    
                    for (size_t i = 0; i < inlier_indices.size(); ++i) {
                        X_inliers(i, 0) = 1.0;
                        X_inliers(i, 1) = X[inlier_indices[i]];
                        y_inliers(i) = y[inlier_indices[i]];
                    }
                    
                    coefficients_ = (X_inliers.transpose() * X_inliers).ldlt().solve(X_inliers.transpose() * y_inliers);
                }
            }
        }
    }

    std::vector<double> predict(const std::vector<double>& X) {
        std::vector<double> predictions;
        predictions.reserve(X.size());
        
        for (double x : X) {
            predictions.push_back(coefficients_(0) + coefficients_(1) * x);
        }
        
        return predictions;
    }
};

int main() {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Load NPY files
    cnpy::NpyArray lane_labels = cnpy::npy_load("slam06_laneline.npy");
    cnpy::NpyArray point_cloud = cnpy::npy_load("slame06_mapping_global.npy");

    // Get frame 0 data
    float* frame0_data = point_cloud.data<float>() + 0; // First frame
    uint8_t* labels_data = lane_labels.data<uint8_t>() + 0; // First frame

    // Extract lane points
    std::vector<float> lane_points;
    size_t num_points = 128 * 1024; // Same as Python reshape
    
    for (size_t i = 0; i < num_points; ++i) {
        if (labels_data[i] == 1) {
            // Store x, y coordinates for lane points
            lane_points.push_back(frame0_data[i * 7]);     // x
            lane_points.push_back(frame0_data[i * 7 + 1]); // y
            lane_points.push_back(frame0_data[i * 7 + 2]); // z
        }
    }

    // Perform clustering
    LaneLineClusterer clusterer(1.0, 2);
    auto clusters = clusterer.clusterPoints(lane_points.data(), lane_points.size() / 3);

    auto cluster_time = std::chrono::high_resolution_clock::now();
    std::cout << "Time taken for clustering: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(cluster_time - start_time).count() / 1000.0 
              << " seconds" << std::endl;

    // Fit RANSAC to each cluster
    std::map<int, std::pair<std::vector<double>, std::vector<double>>> fitted_lines;
    
    for (size_t cluster_id = 0; cluster_id < clusters.size(); ++cluster_id) {
        // Extract cluster points
        std::vector<double> X_cluster, Y_cluster;
        for (int idx : clusters[cluster_id]) {
            X_cluster.push_back(lane_points[idx * 3]);     // x
            Y_cluster.push_back(lane_points[idx * 3 + 1]); // y
        }

        // Fit RANSAC
        RANSACRegressor ransac;
        ransac.fit(X_cluster, Y_cluster);

        // Generate points for the fitted line
        double x_min = *std::min_element(X_cluster.begin(), X_cluster.end());
        double x_max = *std::max_element(X_cluster.begin(), X_cluster.end());
        double map_length = x_max - x_min;
        int points_per_meter = 10;
        int num_points = static_cast<int>(map_length * points_per_meter);

        std::vector<double> X_range;
        for (int i = 0; i < num_points; ++i) {
            X_range.push_back(x_min + (x_max - x_min) * i / (num_points - 1));
        }

        std::vector<double> Y_pred = ransac.predict(X_range);
        fitted_lines[cluster_id] = std::make_pair(X_range, Y_pred);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Time taken for RANSAC: " 
              << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - cluster_time).count() / 1000.0 
              << " seconds" << std::endl;

    // Optional: Save results or visualize
    // Note: Visualization would require additional libraries like OpenCV or PCL

    return 0;
}