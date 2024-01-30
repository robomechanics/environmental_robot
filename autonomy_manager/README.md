# Manager 

## State Machine
```mermaid
flowchart TD;
    STANDBY --> WAITING_FOR_CALIBRATION_TO_FINISH;
    WAITING_FOR_CALIBRATION_TO_FINISH --> STANDBY_INIT?;
    STANDBY_INIT? --> RECEIVED_SEARCH_AREA;
    RECEIVED_SEARCH_AREA --> |adaptive| RUNNING_SEARCH_ALGO --> RECEIVED_NEXT_SCAN_LOC --> NAVIGATION_TO_SCAN_LOC;
    RECEIVED_SEARCH_AREA --> |grid| RUNNING_GRID_ALGO --> RECEIVED_NEXT_SCAN_LOC --> NAVIGATION_TO_SCAN_LOC;
    RECEIVED_SEARCH_AREA --> |waypoint| RUNNING_WAYPOINT_ALGO --> RECEIVED_NEXT_SCAN_LOC --> NAVIGATION_TO_SCAN_LOC;
    NAVIGATION_TO_SCAN_LOC --> FINISHED_RAKING --> FINISHED_SCAN;
    
```


## Code Logic
- DeployAutonomy is a list of (lat, lan) and returns success if path was successfully followed

---
# Scripts in Autonomy Manager
- adaptiveROS.py:
  - Adaptive Sampling Algorithm
- autonomy_teleop.py
  - defunct and replaced by autonomy_teleop_instant.py
- autonomy_teleop_instant.py
  - teleop
- boundaryCheck.py:
  - Contains one function __boundaryCheck()__
  - used in adaptiveROS.py, boundaryConversion.py and gridROS.py
- boundaryConversion.py
  - GPS related functions
  - used in manager.py, dataParser.py, test_adaptive.py and test.py
- bugAlgo.py
  - Bug Obstacle avoidance
  - has a function __forward_section()__ to get the forward obstacle section of the obstacle map, which is used in ObsAvoidance.py
  - the bug obstacle_avoidance() function seems to not be used @medium
- calibration.py
  - for fetching initial from GPS and some driving
  - could be replaced by the new RTK GPS module
  - run from the GUI and manager.py runs after calibration
- curve_graph.py
  - example code to plot Adaptive vs Boustrophedon
  - y label = EMD, not sure how they EMD values @medium
  - deprecated
- dataParser.py
  - sample code for using visualizer but the function __visualizer()__ is used only inside this file @medium
  - deprecated
- dummy_services.py
  - dummy services to emulate robot? @medium
  - deprecated
- environmentGeneration.py
  - functions to generate random distributions and random obstacles
  - __generateRandomDistribution()__ used in dataParser.py
- gridROS.py
  - Grid Search Algorithm
  - used in manager.py
- manager.py
  - State Machine
- ObsAvoidance.py
  - Obstacle Avoidance Algorithm
- postProcessing.py
  - plotting function __visualizer()__ used in adaptiveROS.py
  - Plotter 
  - is is used during robot execution or after? @medium
- teleop_drive.py
  - Teleop
    - only telop without autonomy teleop
    - deprecated
- test.py
  - test functions in boundaryConversion.py
  - deprecated
- test_adaptive.py
  - test adaptive in simulation @medium
  - 
- todo.md

# environmental_robots
## camera (Removed)
- camera.py
  - HEBI camera controller for Loaner HEBI robot
- hebi_teleop.py
  - HEBI teleop which might be something we don't use anymore
  - refer for arm helper functions
- pan_tile_ctrl.py
  - Loaner HEBI robot camera
- tools_arms_ctrl.py
  - Arm tool control not used
  - refer for arm helper functions

## gps_navigation
- gps_user_input.py
  - actual GUI code
  - Ian made changes for services and topics
- gps_navigation_rviz.py
  - might be defunct and not used
  - removed
- gps_navigation.py
  - future version of gps_navigation_rviz.py?
  - might be defunct and not used
  - some kind of PID controller to go to a location
  - removed
- gps_user_location.py
  - __read_location()__ is used by gps_user_input.py
- rake_measure.py
  - defunct @high
- test.py
  - some test code for parking brake
  - defunct @high
- tile.py
  - used in gps_user_input.py
- Removed all scripts
- Removed kraton and rake_scan pkgs
- 
kraton:
- camera pkg probably not needed @medium


autonomy_manager, gps_user_input.py and pxrf.py
- pxrf change timings

# Tomorrow
- PXRF
  - pxrf scan
  - store in excel
  - add ros timing