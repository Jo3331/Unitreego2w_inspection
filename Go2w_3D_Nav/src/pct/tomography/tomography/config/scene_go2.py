from .scene import Scene


class SceneGo2(Scene):
    """Baseline parameters for a Go2 corridor map from Elevator-LIO."""

    def __init__(self):
        super().__init__()
        # Floor-leveled map: floor is approximately z=0 and the radar origin
        # is approximately 1.2 m above it.
        self.pcd.file_name = 'go2w_mid360_signed_normals.pcd'

        self.map.resolution = 0.10
        self.map.ground_h = 0.0  # 旧值 -1.0（贴 8F 平台中位 -0.498）
        self.map.ground_min_h = -0.3
        self.map.ground_max_h = 9.0
        self.map.normal_ground_min_z = 0.0
        # Keep vertical sampling fine enough to identify the walking surface.
        # A very large value makes the tomography kernel select the highest
        # return in each XY cell, which can be the ceiling.
        self.map.slice_dh = 0.15

        self.trav.kernel_size = 3
        # Stair traversal must remain stricter than the earlier diagnostic
        # configuration, otherwise adjacent slabs are all treated as routes.
        self.trav.interval_min = 0.30
        self.trav.interval_free = 0.45
        self.trav.slope_max = 0.95
        self.trav.step_max = 0.26
        self.trav.standable_ratio = 0.0
        self.trav.cost_barrier = 50.0
        self.trav.safe_margin = 0.05
        self.trav.inflation = 0.04  # 旧值 0.05（取整膨胀半径归 0，救 8F）
        # Set this to a known lower-floor point in map coordinates when the
        # capture origin is known. None uses the lowest valid support surface.
        self.trav.connectivity_seed = None
        self.trav.connectivity_threshold = 20.0
