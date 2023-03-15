# gps_navigation
The gps_navigation package contains the source code for waypoint navigation and GUI.
To launch: please run `roslaunch gps_naviation navigate_no_rviz.launch`

The following files in the src are used:

	gps_user_location.py:
It both parses the locations.csv to extract the coordinate information. It can also take the input from the user and store the coordinate in the locations.csv file so that it can be easily accessed in the future

	gps_user_input.py:
The main GUI that uses PyQT as the library that displays the regional map. It allows the user to place pins on the map, control pxrf, monitor robot status, etc

	gps_navigation.py:
The main controller for waypoint navigation. It includes both LQR and PID controllers. Currently only PID controller is used. LQR controller is not well tuned.


