# Manager 

## Code Logic
- SetSearchBoundary is a list of (lat,lon) for gps and (x,y) for map and returns success if path was successfully followed

---
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
- environmentGeneration.py
  - functions to generate random distributions and random obstacles
  - __generateRandomDistribution()__ used in dataParser.py
- gridROS.py
  - Grid Search Algorithm
  - used in manager.py
- manager.py
  - State Machine
- postProcessing.py
  - plotting function __visualizer()__ used in adaptiveROS.py
  - Plotter 
  - is is used during robot execution or after? @medium

# environmental_robots
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


# TODO
- WAITING_FOR_CALIBRATION_TO_FINISH: Need to wait for RTK GPS to be initialized. May be change it to WAITING_FOR_GPS_INIT
- the first STANDBY state can be changed to INIT