# Jetson Environment
## System Info
### Power
9.0V – 19.6V
# Network Information
## HEBI Arm
It is connected to eth0 and uses a static IP configuration.

```
ROBOT IP, Netmask, Gateway: 192.168.0.200, 24, 192.168.0.1
HEBI IP : 192.168.0.102
```

## PXRF
It is connected to usb1, a USB ethernet module. It might use DHCP.
```
ROBOT IP, Netmask, Gateway: 192.168.7.10, 24, 192.168.7.1
PXRF IP : 192.168.7.2
```

## Terminal Info
- zsh is the default terminal. 
- Instead of .bashrc, use .zshrc file to add aliases, source rosfiles, etc
- Run `tmux` after ssh to easily create new tmux session. A tmux session allows you to create remote terminals wihtout having to repeatedly ssh again.

## Aliases
The following aliases use incremental search to list the packages, topics and nodes.
- rcd -> roscd
- rte -> rostopic echo
- rtinfo -> rostopic info
- rnkill -> rosnode kill

## Shortcuts:
- CTRL + Space : Accept autosuggestion in the terminal
- CTRL + L : Clear terminal
- CTRL + R : Incrementally search history
- ` is the prefix key for tmux. After pressing the prefix key, press the following for:
  - | : Split current terminal horizontally
  - -- : Split current temrinal vertically
  - c : New terminal
  - x : Close current terminal
  - s : Switch session


# Links
1. [Flipper Rover Pro](https://roverrobotics.com/products/flipper-pro-unmanned-ground-vehicle-ros2-robot)
2. [RTK GPS](https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/user_manual_content/RTK/3DM%20RTK%20Integration.htm)
3. [Clifford Parts for RTK GPS](https://cmu.app.box.com/file/983972334542?s=4fbow8q6s7l7qhiz32eurrs9au0b984r)
4. [Sparkfun Basetation](https://www.sparkfun.com/products/retired/19029)
5. [PXRF repo](https://github.com/robomechanics/PXRF)
6. [PXRF Vanta Specs](https://www.olympus-ims.com/en/xrf-analyzers/handheld/vanta/#!cms[focus]=cmsContent14332)
7. [Remote SSH Code Server](https://code.visualstudio.com/docs/remote/ssh)
8. [Hebi Python API](http://docs.hebi.us/tools.html#python-api)
9. [Measuring Antenna Offsets](https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/user_manual_content/installation/Antenna.htm#How)
10. [Calibrating Magnetometer](https://s3.amazonaws.com/files.microstrain.com/GQ7+User+Manual/user_manual_content/installation/Magnetometer%20Calibration.htm)
11. [GQ7 Module Specs](https://www.microstrain.com/sites/default/files/8400-0139%20REV%20B.pdf)
