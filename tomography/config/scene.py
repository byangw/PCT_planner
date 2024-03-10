class ScenePCD():
    file_name = None


class SceneMap():
    resolution = 0.10
    ground_h = 0.0
    slice_dh = 0.5


class SceneTrav():
    kernel_size = 7
    interval_min = 0.50
    interval_free = 0.65
    slope_max = 0.36
    step_max = 0.20
    standable_ratio = 0.20
    cost_barrier = 50.0

    safe_margin = 0.4
    inflation = 0.2