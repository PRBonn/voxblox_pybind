#!/usr/bin/env python3
# @file      kitti_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from kitti_dataset import KITTIOdometryDataset as Dataset
from tsdf_pipeline import TSDFPipeline


def main(
    kitti_root_dir: str,
    sequence: int = 0,
    config: str = "config/kitti.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    map_name = f"kitti_{sequence}_scans_{str(n_scans)}"
    dataset = Dataset(kitti_root_dir, sequence, config)
    pipeline = TSDFPipeline(dataset, config, jump, n_scans, map_name)
    pipeline.run()
    pipeline.draw_mesh() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
