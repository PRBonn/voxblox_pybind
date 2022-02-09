#!/usr/bin/env python3
# @file      nuscenes_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from nuscenes import NUSCENESDataset as Dataset
from tsdf_pipeline import TSDFPipeline


def main(
    nusc_root_dir: str,
    sequence: int = 0,
    config: str = "config/kitti.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(nusc_root_dir, sequence, config)
    pipeline = TSDFPipeline(dataset, config, jump, n_scans, map_name=f"nuscenes_{sequence}")
    pipeline.run()
    pipeline.draw_mesh() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
