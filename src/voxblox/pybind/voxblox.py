from abc import ABC, abstractmethod
from typing import Dict

import numpy as np

from . import voxblox_pybind
from .config import __default_config__


class BaseTsdfIntegrator(ABC):
    @abstractmethod
    def __init__(self, voxel_size: float, sdf_trunc: float, config: Dict = __default_config__):
        self.voxel_size = np.float32(voxel_size)
        self.sdf_trunc = np.float32(sdf_trunc)
        self.config = config

    def integrate(self, points: np.ndarray, extrinsic: np.ndarray):
        """TODO: Add docstring here..."""
        assert isinstance(points, np.ndarray), "points must by np.ndarray(n, 3)"
        assert isinstance(extrinsic, np.ndarray), "extrinsic must by np.ndarray(4, 4)"
        assert points.dtype == np.float64, "points dtype must be np.float64"
        assert (
            extrinsic.dtype == np.float or extrinsic.dtype == np.float32
        ), "extrinsic dtype must be np.float32 or np.float64"
        self._integrator._integrate(voxblox_pybind._VectorEigen3d(points), extrinsic)

    def extract_triangle_mesh(self):
        """Returns a the vertices and triangles representing the constructed the TriangleMesh.

        If you can afford to use Open3D as dependency just pass the output of this function to the
        TriangleMesh constructor from Open3d.

        vertices, triangles = integrator.extract_triangle_mesh()
        mesh = o3d.geometry.TriangleMesh(
            o3d.utility.Vector3dVector(vertices),
            o3d.utility.Vector3iVector(triangles),
        )
        """
        vertices, triangles = self._integrator._extract_triangle_mesh()
        return np.asarray(vertices), np.asarray(triangles)


class SimpleTsdfIntegrator(BaseTsdfIntegrator):
    def __init__(self, voxel_size: float, sdf_trunc: float, config: Dict = __default_config__):
        super().__init__(voxel_size, sdf_trunc, config)
        self._integrator = voxblox_pybind._SimpleTsdfIntegrator(
            voxel_size=self.voxel_size, sdf_trunc=self.sdf_trunc, config=self.config
        )


class FastTsdfIntegrator(BaseTsdfIntegrator):
    def __init__(self, voxel_size: float, sdf_trunc: float, config: Dict = __default_config__):
        super().__init__(voxel_size, sdf_trunc, config)
        self._integrator = voxblox_pybind._FastTsdfIntegrator(
            voxel_size=self.voxel_size, sdf_trunc=self.sdf_trunc, config=self.config
        )


class MergedTsdfIntegrator(BaseTsdfIntegrator):
    def __init__(self, voxel_size: float, sdf_trunc: float, config: Dict = __default_config__):
        super().__init__(voxel_size, sdf_trunc, config)
        self._integrator = voxblox_pybind._MergedTsdfIntegrator(
            voxel_size=self.voxel_size, sdf_trunc=self.sdf_trunc, config=self.config
        )
