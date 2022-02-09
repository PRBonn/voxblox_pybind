#!/usr/bin/env python3
# @file      cow_pipeline.py
# @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
#
# Copyright (c) 2021 Ignacio Vizzo, all rights reserved
import argh

from cow import CowDataset as Dataset
from tsdf_pipeline import TSDFPipeline


def main(
    data_source: str,
    config: str = "config/cow.yaml",
    n_scans: int = -1,
    jump: int = 0,
    visualize: bool = False,
):
    """Help here!"""
    dataset = Dataset(data_source, get_color=False, apply_pose=False)
    pipeline = TSDFPipeline(dataset, config, jump, n_scans, f"cow_scans_{str(n_scans)}")
    pipeline.run()
    pipeline.draw_mesh() if visualize else None


if __name__ == "__main__":
    argh.dispatch_command(main)
