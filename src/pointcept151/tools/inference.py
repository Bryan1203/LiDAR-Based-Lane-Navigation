import torch
import numpy as np
import torch.nn.functional as F
from pointcept.models.point_transformer_v3.point_transformer_v3m1_base import PointTransformerV3
from pointcept.datasets.transform import (
    GridSample, Compose, ToTensor, Collect, RandomRotateTargetAngle
)
from pointcept.datasets.utils import collate_fn

class PointTransformerInference:
    def __init__(self, weight_path, confidence_threshold=0.5):
        self.confidence_threshold = confidence_threshold
        
        # Initialize model
        self.setup_model(weight_path)
        
        # Setup transforms from test config
        self.voxelize = GridSample(
            grid_size=0.05,
            hash_type='fnv',
            mode='test',
            return_grid_coord=True,
            keys=('coord', 'strength')
        )
        
        self.post_transform = Compose([
            ToTensor(),
            Collect(
                keys=('coord', 'grid_coord', 'index'),
                feat_keys=('coord', 'strength')
            )
        ])
        
        self.aug_transform = [
            Compose([
                RandomRotateTargetAngle(
                    angle=[0],
                    axis='z',
                    center=[0, 0, 0],
                    p=1
                )
            ])
        ]
        
    def setup_model(self, weight_path):
        """Initialize Point Transformer model"""
        # Create backbone
        self.backbone = PointTransformerV3(
            in_channels=4,
            order=['z', 'z-trans', 'hilbert', 'hilbert-trans'],
            stride=(2, 2, 2, 2),
            enc_depths=(2, 2, 2, 6, 2),
            enc_channels=(32, 64, 128, 256, 512),
            enc_num_head=(2, 4, 8, 16, 32),
            # enc_patch_size=(128, 128, 128, 128, 128),
            enc_patch_size=(1024, 1024, 1024, 1024, 1024),
            dec_depths=(2, 2, 2, 2),
            dec_channels=(64, 64, 128, 256),
            dec_num_head=(4, 4, 8, 16),
            # dec_patch_size=(128, 128, 128, 128),
            dec_patch_size=(1024, 1024, 1024, 1024),
            mlp_ratio=4,
            qkv_bias=True,
            qk_scale=None,
            attn_drop=0.0,
            proj_drop=0.0,
            drop_path=0.3,
            shuffle_orders=True,
            pre_norm=True,
            enable_rpe=False,
            enable_flash=False,
            upcast_attention=False,
            upcast_softmax=False,
            cls_mode=False,
            pdnorm_bn=False,
            pdnorm_ln=False,
            pdnorm_decouple=True,
            pdnorm_adaptive=False,
            pdnorm_affine=True,
            pdnorm_conditions=('nuScenes', 'SemanticKITTI', 'Waymo')
        )

        # Create segmentation head
        self.seg_head = torch.nn.Sequential(
            torch.nn.Linear(64, 64),
            torch.nn.BatchNorm1d(64),
            torch.nn.ReLU(inplace=True),
            torch.nn.Linear(64, 2)  # 2 classes
        )

        # Load weights
        checkpoint = torch.load(weight_path)
        self.backbone.load_state_dict(checkpoint["backbone"], strict=True)
        self.seg_head.load_state_dict(checkpoint["seg_head"], strict=True)
        
        self.backbone.cuda()
        self.seg_head.cuda()
        self.backbone.eval()
        self.seg_head.eval()
        
        print(f"Model loaded from {weight_path}")

    def forward(self, data_dict):
        """Forward pass through the model"""
        point = self.backbone(data_dict)
        logits = self.seg_head(point.feat)
        return {"seg_logits": logits}

    def prepare_input(self, points):
        """Prepare point cloud input"""
        # Create initial data dictionary
        data_dict = {
            'coord': points[:, :3],
            'strength': points[:, 3].reshape(-1, 1),
            'segment': np.zeros(len(points), dtype=np.int32)
        }
        
        # Apply voxelization
        data_dict = self.voxelize(data_dict)
        
        # Apply augmentation transforms
        data_dict_list = []
        for aug in self.aug_transform:
            data_dict_aug = data_dict.copy()
            data_dict_aug = aug(data_dict_aug)
            data_dict_list.append(data_dict_aug)
        
        # Apply post transforms
        fragment_list = []
        for data_dict_aug in data_dict_list:
            fragment = self.post_transform(data_dict_aug)
            fragment_list.append(fragment)
        
        return fragment_list, points.shape[0]

    def __call__(self, points):
        """
        Perform inference on point cloud
        Args:
            points: numpy array of shape (N, 4) with columns [x, y, z, intensity]
        Returns:
            lane_points: numpy array of shape (M, 5) with columns [x, y, z, intensity, confidence]
            where M is the number of points classified as lane
        """
        assert isinstance(points, np.ndarray) and points.shape[1] == 4, \
            "Input should be numpy array of shape (N, 4) [x, y, z, intensity]"
        
        # Prepare input with transforms
        fragment_list, segment_size = self.prepare_input(points)
        
        # Initialize prediction tensor
        pred = torch.zeros((segment_size, 2)).cuda()
        
        # Process each fragment
        for fragment in fragment_list:
            batch = collate_fn([fragment])
            
            # Move to GPU
            for key in batch.keys():
                if isinstance(batch[key], torch.Tensor):
                    batch[key] = batch[key].cuda(non_blocking=True)
            
            idx_part = batch["index"]
            
            # Perform inference
            with torch.no_grad():
                pred_part = self.forward(batch)["seg_logits"]
                pred_part = F.softmax(pred_part, -1)
            
            # Accumulate predictions
            bs = 0
            for be in batch["offset"]:
                pred[idx_part[bs:be], :] += pred_part[bs:be]
                bs = be
        
        # Average predictions from augmentations
        pred = pred / len(self.aug_transform)
        
        # Get final predictions
        pred_labels = pred.max(1)[1].cpu().numpy()
        pred_scores = pred.max(1)[0].cpu().numpy()
        
        # Filter lane line points
        lane_mask = (pred_labels == 1) & (pred_scores >= self.confidence_threshold)
        
        if np.sum(lane_mask) == 0:
            print("No lane lines detected")
            return None
        
        # Create output points array
        lane_points = np.zeros((np.sum(lane_mask), 5), dtype=np.float32)
        lane_points[:, :4] = points[lane_mask, :4]  # xyz and intensity
        lane_points[:, 4] = pred_scores[lane_mask]  # confidence scores
        
        print(f"Detected {len(lane_points)} lane points out of {len(points)} total points")
        
        return lane_points

# Example usage:
if __name__ == "__main__":
    # Load some test points (replace with your actual data loading)
    test_points = np.random.rand(1000, 4)  # Random test data
    
    # Initialize model
    model = PointTransformerInference(
        weight_path="/home/yan/pointcept151/exp/nuscenes/train_highbay_02/model/model_best.pth",
        confidence_threshold=0.5
    )
    
    # Run inference
    lane_points = model(test_points)
    
    if lane_points is not None:
        print("Lane points shape:", lane_points.shape)
        print("Lane points example:")
        print(lane_points[:5])  # Print first 5 detected lane points