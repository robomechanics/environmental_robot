import rospy
from std_msgs.msg import String


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
    def __init__(self, data_response_callback=None):
        self.load_ros_params()
        
        self.scanning = False
        self.scan_completed = False
        self.data_response_callback = data_response_callback
        
        self.pxrf_command_pub = rospy.Publisher(self._pxrf_cmd_topic, String, queue_size=1)
        self._pxrf_response_sub = rospy.Subscriber(self._pxrf_response_topic, String, self.pxrf_response_callback)

    def load_ros_params(self):
        self._pxrf_cmd_topic = rospy.get_param("pxrf_cmd_topic")
        self._pxrf_response_topic = rospy.get_param("pxrf_response_topic")

    def start_scan(self):
        self.scanning = True
        self.scan_completed = False
        self.pxrf_command_pub.publish("start")
        
    def stop_scan(self):
        self.pxrf_command_pub.publish("stop")
        self.scanning = False

    def pxrf_response_callback(self, data):
        # rospy.loginfo("Test complete")
        self.scan_completed = (data.data == "201")
        
        if self.data_response_callback and self.scan_completed:
            self.data_response_callback(data)
            self.scan_completed = False
        


if __name__ == '__main__':
    rospy.init_node('pxrf_handler',anonymous=True)
    pxrf = PXRF()
    pxrf.start_scan()
    rospy.spin()
