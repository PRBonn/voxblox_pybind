"""Test the C++/Python bindings."""
import unittest

import numpy as np

from voxblox import (
    BaseTsdfIntegrator,
    FastTsdfIntegrator,
    MergedTsdfIntegrator,
    SimpleTsdfIntegrator,
)


class VoxbloxPybindTest(unittest.TestCase):
    """Since this is not my real code, I won't provide any meaningfull test.

    I will just check that the API is working and that it doesn't crash right away. If the
    internal code is not doing what is expected, then the original voxblox library should
    be checked.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.voxel_size: float = 0.1
        self.sdf_trunc: float = 0.3
        self.n_points = 100
        self.points = np.random.rand(self.n_points, 3)
        self.pose = np.eye(4)

    def _test_integrate(self, integrator: BaseTsdfIntegrator) -> None:
        """We can't check nothing meaningfull from the python side, so move on."""
        integrator.integrate(points=self.points, extrinsic=self.pose)

    def _test_extract_tirangle_mesh(self, integrator: BaseTsdfIntegrator) -> None:
        integrator.integrate(points=self.points, extrinsic=self.pose)
        vertices, triangles = integrator.extract_triangle_mesh()
        # We can't see what's inside, but at least should be not None objects
        self.assertIsNotNone(vertices)
        self.assertIsNotNone(triangles)

    def _test_integrator(self, integrator: BaseTsdfIntegrator) -> None:
        self.assertIsNotNone(integrator.config)
        self.assertAlmostEqual(integrator.voxel_size, self.voxel_size)
        self.assertAlmostEqual(integrator.sdf_trunc, self.sdf_trunc)
        self._test_integrate(integrator)
        self._test_extract_tirangle_mesh(integrator)

    def test_fast_tsdf_integrator(self):
        self._test_integrator(
            FastTsdfIntegrator(voxel_size=self.voxel_size, sdf_trunc=self.sdf_trunc)
        )

    def test_merged_tsdf_integrator(self):
        self._test_integrator(
            MergedTsdfIntegrator(voxel_size=self.voxel_size, sdf_trunc=self.sdf_trunc)
        )

    def test_simple_tsdf_integrator(self):
        self._test_integrator(
            SimpleTsdfIntegrator(voxel_size=self.voxel_size, sdf_trunc=self.sdf_trunc)
        )


if __name__ == "__main__":
    unittest.main()
