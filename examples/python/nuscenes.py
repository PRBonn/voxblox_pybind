import glob
import os

from easydict import EasyDict
import numpy as np
import open3d as o3d
import pandas as pd
import yaml


class NUSCENESDataset:
    def __init__(self, nusc_root_dir: str, sequence: int, config_file: str):
        """Simple NUSCENES DataLoader to provide a ready-to-run example.

        Heavily inspired in PyLidar SLAM
        """
        # Config stuff
        self.sequence = str(int(sequence)).zfill(4)
        self.config = EasyDict(yaml.safe_load(open(config_file)))
        self.nusc_sequence_dir = os.path.join(nusc_root_dir, self.sequence)
        self.velodyne_dir = os.path.join(self.nusc_sequence_dir, "velodyne/")

        # Read stuff
        self.calibration = self.read_calib_file(os.path.join(self.nusc_sequence_dir, "calib.txt"))
        self.poses = self.load_poses(os.path.join(self.nusc_sequence_dir, "poses.txt"))
        self.scan_files = sorted(glob.glob(self.velodyne_dir + "*.bin"))

    def __getitem__(self, idx):
        return self.scans(idx), self.poses[idx]

    def __len__(self):
        return len(self.scan_files)

    def scans(self, idx):
        return self.read_point_cloud(idx, self.scan_files, self.config)

    def read_point_cloud(self, idx: int, scan_files: list, config: dict):
        file_path = scan_files[idx]
        points = np.fromfile(file_path, dtype=np.float32).reshape((-1, 4))[:, :3]
        scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        scan = scan.voxel_down_sample(config.voxel_size) if config.voxelize else scan
        points = np.asarray(scan.points)
        points = points[np.linalg.norm(points, axis=1) <= config.max_range]
        points = points[np.linalg.norm(points, axis=1) >= config.min_range]
        scan = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
        scan.transform(self.poses[idx]) if config.apply_pose else None
        return np.asarray(scan.points)

    def load_poses(self, poses_file):
        def _lidar_pose_gt(poses_gt):
            _tr = self.calibration["Tr"].reshape(3, 4)
            tr = np.eye(4, dtype=np.float64)
            tr[:3, :4] = _tr
            left = np.einsum("...ij,...jk->...ik", np.linalg.inv(tr), poses_gt)
            right = np.einsum("...ij,...jk->...ik", left, tr)
            return right

        poses = pd.read_csv(poses_file, sep=" ", header=None).values
        n = poses.shape[0]
        poses = np.concatenate(
            (poses, np.zeros((n, 3), dtype=np.float32), np.ones((n, 1), dtype=np.float32)), axis=1
        )
        poses = poses.reshape((n, 4, 4))  # [N, 4, 4]
        return _lidar_pose_gt(poses)

    @staticmethod
    def read_calib_file(file_path: str) -> dict:
        calib_dict = {}
        with open(file_path, "r") as calib_file:
            for line in calib_file.readlines():
                tokens = line.split(" ")
                if tokens[0] == "calib_time:":
                    continue
                # Only read with float data
                if len(tokens) > 0:
                    values = [float(token) for token in tokens[1:]]
                    values = np.array(values, dtype=np.float32)

                    # The format in KITTI's file is <key>: <f1> <f2> <f3> ...\n -> Remove the ':'
                    key = tokens[0][:-1]
                    calib_dict[key] = values
        return calib_dict
