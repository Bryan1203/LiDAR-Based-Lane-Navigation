"""
Semantic KITTI dataset

Author: Xiaoyang Wu (xiaoyang.wu.cs@gmail.com)
Please cite our work if the code is helpful to you.
"""

import os
import numpy as np

from .builder import DATASETS
from .defaults import DefaultDataset


@DATASETS.register_module()
class SemanticKITTIDataset(DefaultDataset):
    def __init__(
        self,
        split="train",
        data_root="/media/changbryan/BC_T7/itriDataset/",
        transform=None,
        test_mode=False,
        test_cfg=None,
        loop=1,
        ignore_index=-1,
    ):
        self.ignore_index = ignore_index
        self.learning_map = self.get_learning_map(ignore_index)
        self.learning_map_inv = self.get_learning_map_inv(ignore_index)
        super().__init__(
            split=split,
            data_root=data_root,
            transform=transform,
            test_mode=test_mode,
            test_cfg=test_cfg,
            loop=loop,
        )

    def get_data_list(self):
        # # Generate sequence numbers 0-406
        # all_sequences = list(range(407))
        
        # # Randomly shuffle the sequences
        # np.random.seed(42)  # For reproducibility
        # np.random.shuffle(all_sequences)
        
        # # Split into train (60%), val (20%), test (20%)
        # train_size = int(0.7 * len(all_sequences))
        # val_size = int(0.15 * len(all_sequences))
        
        split2seq = dict(
        #     train=all_sequences[:train_size],
        #     val=all_sequences[train_size:train_size + val_size],
        #     test=all_sequences[train_size + val_size:],
            train=[00],  # Commented out debug values
            val=[00],
            test=[00],
        )
        
        if isinstance(self.split, str):
            seq_list = split2seq[self.split]
        elif isinstance(self.split, list):
            seq_list = []
            for split in self.split:
                seq_list += split2seq[split]
        else:
            raise NotImplementedError

        data_list = []
        for seq in seq_list:
            seq = str(seq).zfill(2)
            seq_folder = os.path.join(self.data_root, "dataset", "sequences", seq)
            seq_files = sorted(os.listdir(os.path.join(seq_folder, "velodyne")))
            
            # Determine the number of frames for each split
            num_files = len(seq_files)
            num_train = int(num_files * 0.7)
            num_val = int(num_files * 0.15)
            num_test = num_files - num_train - num_val  # Remaining frames for test

            # Split frames within each sequence
            frame_sequences = np.array(range(num_files))
            np.random.seed(42)  # For reproducibility
            np.random.shuffle(frame_sequences)
            train_files = np.take(seq_files, frame_sequences[:num_train])
            val_files = np.take(seq_files, frame_sequences[num_train:num_train + num_val])
            test_files = np.take(seq_files, frame_sequences[num_train + num_val:])

            np.save('frameSequence.npy', frame_sequences)

            # Add files to data_list based on the specified split
            if self.split == "train":
                data_list += [os.path.join(seq_folder, "velodyne", file) for file in train_files]
            elif self.split == "val":
                data_list += [os.path.join(seq_folder, "velodyne", file) for file in val_files]
            elif self.split == "test":
                data_list += [os.path.join(seq_folder, "velodyne", file) for file in test_files]

        return data_list

    def get_data(self, idx):
        data_path = self.data_list[idx % len(self.data_list)]
        with open(data_path, "rb") as b:
            scan = np.fromfile(b, dtype=np.float32).reshape(-1, 4)
        coord = scan[:, :3]
        strength = scan[:, -1].reshape([-1, 1])
        #modified for MT
        #strength = scan[:, -1].reshape([-1, 1])/255.

        label_file = data_path.replace("velodyne", "labels").replace(".bin", ".label")
        if os.path.exists(label_file):
            with open(label_file, "rb") as a:
                segment = np.fromfile(a, dtype=np.int32).reshape(-1)
                segment = np.vectorize(self.learning_map.__getitem__)(
                    segment & 0xFFFF
                ).astype(np.int32)
        else:
            segment = np.zeros(scan.shape[0]).astype(np.int32)
        data_dict = dict(coord=coord, strength=strength, segment=segment)
        return data_dict

    def get_data_name(self, idx):
        file_path = self.data_list[idx % len(self.data_list)]
        dir_path, file_name = os.path.split(file_path)
        sequence_name = os.path.basename(os.path.dirname(dir_path))
        frame_name = os.path.splitext(file_name)[0]
        data_name = f"{sequence_name}_{frame_name}"
        return data_name

    @staticmethod
    def get_learning_map(ignore_index):
        learning_map = {
            # 0: ignore_index,  # "unlabeled"
            # 1: ignore_index,  # "outlier" mapped to "unlabeled" --------------------------mapped
            # 10: 0,  # "car"
            # 11: 1,  # "bicycle"
            # 13: 4,  # "bus" mapped to "other-vehicle" --------------------------mapped
            # 15: 2,  # "motorcycle"
            # 16: 4,  # "on-rails" mapped to "other-vehicle" ---------------------mapped
            # 18: 3,  # "truck"
            # 20: 4,  # "other-vehicle"
            # 30: 5,  # "person"
            # 31: 6,  # "bicyclist"
            # 32: 7,  # "motorcyclist"
            # 40: 8,  # "road"
            # 44: 9,  # "parking"
            # 48: 10,  # "sidewalk"
            # 49: 11,  # "other-ground"
            # 50: 12,  # "building"
            # 51: 13,  # "fence"
            # 52: ignore_index,  # "other-structure" mapped to "unlabeled" ------------------mapped
            # 60: 8,  # "lane-marking" to "road" ---------------------------------mapped
            # 70: 14,  # "vegetation"
            # 71: 15,  # "trunk"
            # 72: 16,  # "terrain"
            # 80: 17,  # "pole"
            # 81: 18,  # "traffic-sign"
            # 99: ignore_index,  # "other-object" to "unlabeled" ----------------------------mapped
            # 252: 0,  # "moving-car" to "car" ------------------------------------mapped
            # 253: 6,  # "moving-bicyclist" to "bicyclist" ------------------------mapped
            # 254: 5,  # "moving-person" to "person" ------------------------------mapped
            # 255: 7,  # "moving-motorcyclist" to "motorcyclist" ------------------mapped
            # 256: 4,  # "moving-on-rails" mapped to "other-vehicle" --------------mapped
            # 257: 4,  # "moving-bus" mapped to "other-vehicle" -------------------mapped
            # 258: 3,  # "moving-truck" to "truck" --------------------------------mapped
            # 259: 4,  # "moving-other"-vehicle to "other-vehicle" ----------------mapped
            


            0: 0,  # "unlabeled"
            # used the labeling from the shp file 
            # map the specific road lines to dahed, solid, and double
            1: 1,  
            # 2: 1,
            # 3: 3,
            # 4: 1,
            # 5: 1,
            # 6: 1,

            # 11: 2,
            # 12: 1,
            # 13: 3,
            # 14: 1,
            # 15: 1,
            # 16: 1,

            # 21: 2,
            # 22: 1,
            # 23: 3,
            # 24: 1,
            # 25: 1,
            # 26: 1,
            # #labeling used in ITRI semantic maps
            # 41: 1,
            # 42: 2,
            # 43: 3,
            # 44: 1,
            # 45: 2,
            # 46: 3,
            # 47: 2,
        }
        return learning_map

    @staticmethod
    def get_learning_map_inv(ignore_index):
        learning_map_inv = {
            # ignore_index: ignore_index,  # "unlabeled"
            # 0: 10,  # "car"
            # 1: 11,  # "bicycle"
            # 2: 15,  # "motorcycle"
            # 3: 18,  # "truck"
            # 4: 20,  # "other-vehicle"
            # 5: 30,  # "person"
            # 6: 31,  # "bicyclist"
            # 7: 32,  # "motorcyclist"
            # 8: 40,  # "road"
            # 9: 44,  # "parking"
            # 10: 48,  # "sidewalk"
            # 11: 49,  # "other-ground"
            # 12: 50,  # "building"
            # 13: 51,  # "fence"
            # 14: 70,  # "vegetation"
            # 15: 71,  # "trunk"
            # 16: 72,  # "terrain"
            # 17: 80,  # "pole"
            # 18: 81,  # "traffic-sign"


            ignore_index: ignore_index,
            0: 0,  # "unlabeled"
            1: 1,  
            # 2: 2,
            # 3: 3,
        }
        return learning_map_inv
