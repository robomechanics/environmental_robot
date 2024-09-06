import rospkg
import subprocess

def ping_ip(ip_address):
    try:
        # Use the ping command with a single packet (-c 1) and a timeout of 1 second (-W 1)
        output = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ip_address],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        # Check the return code to determine if the ping was successful
        return output.returncode == 0
    except Exception as e:
        print(f"An error occurred: {e}")
        return False

def get_ros_pkg_path(pgk_name):
    return rospkg.RosPack().get_path(pgk_name)