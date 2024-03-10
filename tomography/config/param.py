class ConfigROS():
    map_frame = "map"

    pointcloud_topic = "/global_points"
    layer_G_topic = "/layer_G_"
    layer_C_topic = "/layer_C_"
    tomogram_topic = "/tomogram"


class ConfigMap():
    export_dir = "/rsc/tomogram/"


class Config():
    ros = ConfigROS()
    map = ConfigMap()