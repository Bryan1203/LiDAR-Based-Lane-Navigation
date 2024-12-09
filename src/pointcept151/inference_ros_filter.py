#!/usr/bin/env python

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import numpy as np
import torch
import torch.nn.functional as F
from collections import OrderedDict
import time
from pointcept.models import build_model
from pointcept.datasets.transform import (
    GridSample, Compose, ToTensor, Collect, TRANSFORMS
)
torch.backends.cudnn.benchmark = True
torch.backends.cudnn.enabled = True

class PointCloudInference:
    def __init__(self):
        rospy.init_node('point_cloud_inference_node', anonymous=True)
        
        # Filter parameters
        self.height_threshold = -2.0
        self.distance_threshold = 30.0
        self.xy_threshold = 30.0
        self.ring_threshold = 80
        self.range_threshold = 30000
        
        # Model configuration remains the same...
        self.model_cfg = dict(
            type='DefaultSegmentorV2',
            num_classes=2,
            backbone_out_channels=64,
            backbone=dict(
                type='PT-v3m1',
                in_channels=4,
                order=['z', 'z-trans', 'hilbert', 'hilbert-trans'],
                stride=(2, 2, 2, 2),
                enc_depths=(2, 2, 2, 6, 2),
                enc_channels=(32, 64, 128, 256, 512),
                enc_num_head=(2, 4, 8, 16, 32),
                enc_patch_size=(64, 64, 64, 64, 64),
                dec_depths=(2, 2, 2, 2),
                dec_channels=(64, 64, 128, 256),
                dec_num_head=(4, 4, 8, 16),
                dec_patch_size=(64, 64, 64, 64),
                mlp_ratio=4,
                qkv_bias=True,
                qk_scale=None,
                attn_drop=0.0,
                proj_drop=0.0,
                drop_path=0.3,
                shuffle_orders=True,
                pre_norm=True,
                enable_rpe=True,
                enable_flash=False,
                upcast_attention=True,
                upcast_softmax=True,
                cls_mode=False,
                pdnorm_bn=False,
                pdnorm_ln=False,
                pdnorm_decouple=True,
                pdnorm_adaptive=False,
                pdnorm_affine=True,
                pdnorm_conditions=('nuScenes', 'SemanticKITTI', 'Waymo'))
        )

        # Initialize transforms
        self.transform = Compose([
            dict(type='Copy', keys_dict=dict(segment='origin_segment')),
            dict(
                type='GridSample',
                grid_size=0.05,
                hash_type='fnv',
                mode='train',
                keys=('coord', 'strength', 'segment'),
                return_inverse=True)
        ])

        self.voxelize = TRANSFORMS.build(dict(
            type='GridSample',
            grid_size=0.05,
            hash_type='fnv',
            mode='test',
            return_grid_coord=True,
            keys=('coord', 'strength')))

        self.post_transform = Compose([
            dict(type='ToTensor'),
            dict(
                type='Collect',
                keys=('coord', 'grid_coord', 'index'),
                feat_keys=('coord', 'strength'))
        ])

        weight_path = rospy.get_param('~weight_path', '/media/kevin/X9 Pro/484_final_project/models/train_highbay_07/model/model_best.pth')
        self.model = build_model(self.model_cfg)
        if torch.cuda.is_available():
            self.model = self.model.cuda()
            checkpoint = torch.load(weight_path)
        else:
            checkpoint = torch.load(weight_path, map_location='cpu')
        
        weight = OrderedDict()
        for key, value in checkpoint["state_dict"].items():
            if key.startswith("module."):
                key = key[7:]
            weight[key] = value
        self.model.load_state_dict(weight, strict=True)
        self.model.eval()

        self.inference_pub = rospy.Publisher(
            'ouster/points_inference',
            PointCloud2,
            queue_size=10
        )

        self.sub = rospy.Subscriber(
            'ouster/points',
            PointCloud2,
            self.inference_callback,
            queue_size=1
        )
        
        rospy.loginfo("Point cloud inference node initialized")

    def inference_callback(self, msg):
        # try:
            start_time = time.time()
            

            scan_seq = float(msg.header.seq)
            # rospy.loginfo(f"Processing Ouster scan #{scan_seq}")

            sequences = []
            # Filter points
            filtered_points = []
            for point in pc2.read_points(msg, skip_nans=True):
                if point[6] >= self.ring_threshold and point[8]<=self.range_threshold and point[2]<=self.height_threshold:
                    filtered_points.append([point[0], point[1], point[2], point[3]/65532])
                    sequences.append(scan_seq)

            if not filtered_points:
                rospy.logwarn(f"No points passed the filter for scan #{scan_seq}")
                return

            sequences = np.array(sequences, dtype=np.uint32)
            points = np.array(filtered_points)
            filter_time = time.time()
            # rospy.loginfo(f"Filtering time for scan #{scan_seq}: {filter_time - start_time:.3f} seconds")

            # Rest of your inference code remains the same...
            coord = points[:, :3]
            strength = points[:, -1].reshape([-1, 1])
            segment = np.zeros(points.shape[0]).astype(np.int32)
            data_dict = dict(coord=coord, strength=strength, segment=segment)

            data_dict = self.transform(data_dict)
            inverse = data_dict.pop("inverse")
            data_part_list = self.voxelize(data_dict)
            
            fragment_list = []
            for data_part in data_part_list:
                fragment_list.append(self.post_transform(data_part))

            pred = torch.zeros((points.shape[0], 2))
            if torch.cuda.is_available():
                pred = pred.cuda()

            for fragment in fragment_list:
                idx_part = fragment["index"]
                
                input_dict = {}
                for key, value in fragment.items():
                    if isinstance(value, torch.Tensor):
                        input_dict[key] = value.cuda() if torch.cuda.is_available() else value
                    else:
                        input_dict[key] = value

                with torch.no_grad():
                    pred_part = self.model(input_dict)["seg_logits"]
                    pred_part = F.softmax(pred_part, -1)
                    
                    bs = 0
                    for be in input_dict["offset"]:
                        pred[idx_part[bs:be], :] += pred_part[bs:be]
                        bs = be
                
                torch.cuda.empty_cache()

            probs = pred.max(1)[0].cpu().numpy()
            pred = pred.max(1)[1].cpu().numpy()

            pred = pred[inverse]
            probs = probs[inverse]

            points_with_pred = np.column_stack((points, pred, probs, sequences))
            
            fields = [
                PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                PointField(name='prediction', offset=16, datatype=PointField.FLOAT32, count=1),
                PointField(name='confidence', offset=20, datatype=PointField.FLOAT32, count=1),
                PointField(name='sequence', offset=24, datatype=PointField.FLOAT32, count=1),
            ]

            header = std_msgs.msg.Header()
            header.stamp = msg.header.stamp
            # header.seq = msg.header.seq
            # header.child_frame_id = str(scan_seq)
            header.frame_id = msg.header.frame_id
            
            result_msg = pc2.create_cloud(header, fields, points_with_pred)
            self.inference_pub.publish(result_msg)

            end_time = time.time()
            rospy.loginfo(f"Total processing time for scan #{scan_seq}: {end_time - start_time:.3f} seconds")

        # except Exception as e:
        #     rospy.logerr(f"Error during inference: {str(e)}")

def main():
    try:
        node = PointCloudInference()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down point cloud inference node")

if __name__ == '__main__':
    main()
