pc-open-loop() {
    rosnode-kill "/rr_openrover_driver_closed_loop"; 
    sleep 1; 
    roslaunch robo_nav rover_open_loop.launch
}

pc-closed-loop() {
    rosnode-kill "/rr_openrover_driver_open_loop"; 
    sleep 1; 
    roslaunch robo_nav rover_closed_loop.launch
}

alias pc-gps-localization='roslaunch robo_nav localization.launch'
alias pc-basic-bringup='roslaunch autonomy_manager basic.launch'
alias pc-autonomy-bringup='mon launch autonomy_manager bringup.launch --log="/home/patrick/catkin_ws/src/logs/$(date '+%Y-%m-%d-%H:%M:%S').log" --stop-timeout=10'