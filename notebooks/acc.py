import numpy as np

inference_points = np.load('/media/kevin/X9 Pro/484_final_project/processed_point_cloud/slam10_predictions.npy').flatten()
# inference_points = inference_points[:,:, 3]
ground_truth_points = np.load('/media/kevin/X9 Pro/484_final_project/processed_point_cloud/slam10_lane_line_label.npy').flatten()
# ground_truth_points = ground_truth_points[:,:, 3]

inference_positive_indices = inference_points ==1
ground_truth_positive_indices = ground_truth_points ==1

inference_negative_indices = inference_points ==0
ground_truth_negative_indices = ground_truth_points ==0

true_positive = np.sum(inference_positive_indices & ground_truth_positive_indices)
false_positive = np.sum(inference_positive_indices & ground_truth_negative_indices)
true_negative = np.sum(inference_negative_indices & ground_truth_negative_indices)
false_negative = np.sum(inference_negative_indices & ground_truth_positive_indices)

recall = true_positive / (true_positive + false_negative)
precision = true_positive / (true_positive + false_positive)

print('Recall:', recall)
print('Precision:', precision)

