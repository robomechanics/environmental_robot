# Manager 

## Code Logic
- SetSearchBoundary is a list of (lat,lon) for gps and (x,y) for map and returns success if path was successfully followed

#### Publications: 
 * /autonomy_manager/status [autonomy_manager/ManagerStatus]
 * /gps_recorded_before_backup [sensor_msgs/NavSatFix]
 * /pxrf_cmd [std_msgs/String]

#### Subscriptions: 
 * /gq7/ekf/llh_position [unknown type]
 * /pxrf_response [unknown type]
 * /scan_recorded_to_disk [unknown type]

#### Services: 
 * /autonomy_manager/set_search_boundary
 * /clear
 * /manager_run_loop
 * /waypoints

# Scripts in Autonomy Manager
- adaptiveROS.py:
  - Adaptive Sampling Algorithm
- boundaryCheck.py:
  - Contains one function __boundaryCheck()__
  - used in adaptiveROS.py, boundaryConversion.py and gridROS.py
- boundaryConversion.py
  - GPS related functions
  - used in manager.py, dataParser.py, test_adaptive.py and test.py
- dataParser.py
  - sample code for using visualizer but the function __visualizer()__ is used only inside this file @medium
  - deprecated
- gridROS.py
  - Grid Search Algorithm
  - used in manager.py
- manager.py
  - State Machine
- postProcessing.py
  - plotting function __visualizer()__ used in adaptiveROS.py
  - Plotter 
  - is is used during robot execution or after? @medium
- environmentGeneration.py
  - functions to generate random distributions and random obstacles
  - __generateRandomDistribution()__ used in dataParser.py
