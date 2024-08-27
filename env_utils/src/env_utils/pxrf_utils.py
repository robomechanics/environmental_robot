import rospy
from std_msgs.msg import String
import sys
from pxrf.msg import PxrfMsg

def chemistry_parser(chemistry):
    s = chemistry.strip()
    s = s.replace(" ", "")  # remove whitespace
    s = s.replace("\"", "")  # remove random backslashes
    s = s.replace("\\", "")  # remove quotes
    s = s.replace("[", "")  # remove array beginning
    s = s.replace("]", "")  # remove array ending
    s = s.replace("{", "")
    s = s.replace("}", "")
    s = s.replace("\n", "")
    s = s.replace("concentration:", "")
    s = s.replace("elementName:", "")
    s = s.replace("error:", "")
    arr = s.split(",")
    concentration = []
    element = []
    error = []
    for i in range(0, len(arr)):
        if i % 3 == 0:
            arr[i] = float(arr[i])
            concentration.append(arr[i])
        elif i % 3 == 1:
            # string element name
            element.append(arr[i])
        else:  # i % 3 == 2
            arr[i] = float(arr[i])
            error.append(arr[i])
    return element, concentration, error


class PXRF(object):
    def __init__(self, data_callback=None):
        self.load_ros_params()
        
        self.scanning = False
        self.scan_completed = False
        self.data_callback = data_callback
        
        self._pxrf_command_pub = rospy.Publisher(self._pxrf_cmd_topic, String, queue_size=1)
        self._pxrf_response_sub = rospy.Subscriber(self._pxrf_response_topic, String, self.pxrf_response_callback)
        
        if self.data_callback:
            self._pxrf_data_sub = rospy.Subscriber(self._pxrf_data_topic, String, self.pxrf_data_callback)

    def load_ros_params(self):
        self._pxrf_cmd_topic = rospy.get_param("pxrf_cmd_topic")
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")
        self._pxrf_data_topic = rospy.get_param("pxrf_data_topic")

    def start_scan(self):
        self.scanning = True
        self.scan_completed = False
        self._pxrf_command_pub.publish("start")
        
    def stop_scan(self):
        self._pxrf_command_pub.publish("stop")
        self.scanning = False

    def pxrf_response_callback(self, data):
        # rospy.loginfo("Test complete")
        self.scan_completed = (data.data == "201")

    def pxrf_data_callback(self, data: PxrfMsg):
        self.data_callback(data)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python script.py <start|stop>")
        sys.exit(1)

    rospy.init_node('pxrf_handler',anonymous=True)
    pxrf = PXRF()
    
    command = sys.argv[1].lower()

    if command == "start":
        pxrf.start_scan()
    elif command == "stop":
        pxrf.stop_scan()
    
    rospy.spin()