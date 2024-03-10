from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def traj2ros(traj):
    path_msg = Path()
    path_msg.header.frame_id = "map"

    for waypoint in traj:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        pose.pose.orientation.w = 1
        path_msg.poses.append(pose)

    return path_msg