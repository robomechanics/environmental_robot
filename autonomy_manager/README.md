# State Machine
STANDBY -> WAITING_FOR_CALIBRATION_TO_FINISH -> STANDBY (init)
RECEIVED_SEARCH_AREA -> <adaptive/grid/waypoint>
FINISHED_RAKING ->FINISHED_SCAN -> <adaptive/grid/waypoint>
(adaptive) RUNNING_SEARCH_ALGO -> RECEIVED_NEXT_SCAN_LOC -> NAVIGATION_TO_SCAN_LOC
(grid) RUNNING_GRID_ALGO -> RECEIVED_NEXT_SCAN_LOC -> NAVIGATION_TO_SCAN_LOC
(waypoint?) RUNNING_WAYPOINT_ALGO -> RECEIVED_NEXT_SCAN_LOC -> NAVIGATION_TO_SCAN_LOC

---
# Code
DeployAutonomy is a list of (lat, lan) and returns success if path was successfully followed