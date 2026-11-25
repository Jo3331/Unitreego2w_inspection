from .scene_fastliostair import SceneFastliostair


class SceneFastliostairloose(SceneFastliostair):
    """Stair inspection profile that preserves narrow tread observations.

    It retains the oriented-normal test, which rejects downward-facing ceiling
    returns, but disables local support filtering.  Use it to establish a
    traversable baseline before applying a more targeted clearance filter.
    """

    def __init__(self):
        super().__init__()
        # Export under a separate name so the strict-filter tomogram remains
        # available for side-by-side comparison.
        self.pcd.file_name = 'stair_fastlio_gravity_oriented_normals_loose.pcd'
        self.map.ground_support_radius = 0.0
        self.map.ground_support_min_neighbors = 1
        self.map.ground_support_height_tolerance = 0.0
        self.map.ground_support_normal_dot = -1.0
