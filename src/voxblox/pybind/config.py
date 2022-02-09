from dataclasses import asdict, dataclass


@dataclass
class VoxBloxConfig:
    # SimpleTsdfIntegrator
    voxels_per_side: int = 16
    max_weight: float = 10000.0
    voxel_carving_enabled: bool = True
    min_ray_length_m: float = 2.0
    max_ray_length_m: float = 25.0
    use_const_weight: bool = True
    allow_clear: bool = True
    use_weight_dropoff: bool = True
    use_sparsity_compensation_factor: bool = True
    sparsity_compensation_factor: float = 1.0
    integrator_threads: int = 16

    # Custom integrators configs

    # Mode of the ThreadSafeIndex, determines the integration order of the
    # rays. Options: "mixed", "sorted"
    integration_order_mode: str = "mixed"

    # merge integrator specific
    enable_anti_grazing: bool = False

    # fast integrator specific
    start_voxel_subsampling_factor: float = 2.0
    max_consecutive_ray_collisions: int = 2
    clear_checks_every_n_frames: int = 1


__default_config__ = asdict(VoxBloxConfig())
