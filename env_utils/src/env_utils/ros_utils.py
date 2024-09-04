import rospkg

def get_ros_pkg_path(pgk_name):
    return rospkg.RosPack().get_path(pgk_name)