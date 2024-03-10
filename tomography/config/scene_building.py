from .scene import ScenePCD, SceneMap, SceneTrav


class SceneBuilding():
    pcd = ScenePCD()
    pcd.file_name = 'building2_9.pcd'

    map = SceneMap()
    map.resolution = 0.10
    map.ground_h = 0.0
    map.slice_dh = 0.5

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.40
    trav.step_max = 0.17
    trav.standable_ratio = 0.20
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.4
    trav.inflation = 0.2

