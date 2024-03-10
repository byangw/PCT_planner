from .scene import ScenePCD, SceneMap, SceneTrav


class SceneSpiral():
    pcd = ScenePCD()
    pcd.file_name = 'spiral0.3_2.pcd'

    map = SceneMap()
    map.resolution = 0.20
    map.ground_h = 0.0
    map.slice_dh = 0.5

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.40
    trav.step_max = 0.30
    trav.standable_ratio = 0.40
    trav.cost_barrier = 50.0
    trav.safe_margin = 1.2
    trav.inflation = 0.2

