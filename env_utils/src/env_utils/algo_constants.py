ALGO_ADAPTIVE = 'adaptive'
ALGO_GRID = 'grid'
ALGO_WAYPOINT = 'waypoint'
ALGO_MANUAL = 'manual'
ALGO_NONE = 'algo_none'

VALID_ALGOS = [ALGO_ADAPTIVE, ALGO_GRID, ALGO_WAYPOINT, ALGO_MANUAL]

FAKE_ARM = "fake_arm"
FAKE_PXRF = "fake_pxrf"
FAKE_MOVE_BASE = "fake_move_base"
FAKE_HARDWARE_FLAGS = [FAKE_ARM, FAKE_PXRF, FAKE_MOVE_BASE]

INIT = "Initialization"
RECEIVED_SEARCH_AREA = "Received search area"
RECEIVED_NEXT_SCAN_LOC = "Received next scan loc"
ARRIVED_AT_SCAN_LOC = "Arrived at scan loc"
FINISHED_RAKING = "Finished raking"
FINISHED_SCAN = "Finished scan"
RAKING = "Raking"
NAVIGATION_TO_SCAN_LOC = "Navigating to scan loc"
RUNNING_SEARCH_ALGO = "Running search algo"
READY = "Ready"
WAITING_FOR_GPS_INIT = "Waiting for GPS init"
RUNNING_GRID_ALGO = "Running grid algo"
RUNNING_WAYPOINT_ALGO = "Running waypoint algo"
SCANNING = "Scanning"
ARM_RETURNING = "Arm returning"
ARM_LOWERING = "Arm lowering"
ARM_RETURNED = "Arm returned"
ARM_LOWERED = "Arm lowered"
ERROR = "Error"
DONE = "Manager done"
