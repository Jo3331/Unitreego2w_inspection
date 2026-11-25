from .scene import Scene


class SceneFastlio(Scene):
    """PCT profile for the Fast-LIO stair replay with oriented normals.

    The map frame is gravity aligned but its origin is the recording start,
    so the lowest observed floor is near z=-9.25 rather than z=0.
    """

    def __init__(self):
        super().__init__()
        self.pcd.file_name = 'stair_fastlio_gravity_oriented_normals.pcd'
        self.map.resolution = 0.10
        self.map.ground_h = -9.90
        self.map.ground_min_h = -10.00
        self.map.ground_max_h = 1.80
        # Accept only surfaces observed from above.  Ceiling/slab undersides
        # observed by the lidar have normals facing down and stay obstacles.
        self.map.normal_ground_min_z = 0.70
        self.map.slice_dh = 0.12

        self.trav.kernel_size = 7
        self.trav.interval_min = 0.45
        self.trav.interval_free = 0.60
        self.trav.slope_max = 0.70
        self.trav.step_max = 0.24
        self.trav.standable_ratio = 0.12
        self.trav.cost_barrier = 50.0
        self.trav.safe_margin = 0.18
        self.trav.inflation = 0.12
        self.trav.connectivity_enable = False
