# System Packages
```bash
sudo apt install zsh tmux autojump git curl build-essential python3-pip -y

sudo apt-get install net-tools openssh-server python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-noetic-serial ros-noetic-joy ros-noetic-twist-mux ros-noetic-tf2-geometry-msgs ros-noetic-robot-localization ros-noetic-gmapping libqt5websockets5-dev ros-noetic-rtcm-msgs ros-noetic-rosmon ros-noetic-usb-cam ros-noetic-rtcm_msgs ros-noetic-nmea_msgs ros-noetic-mavros
ros-noetic-move-base ros-noetic-move-base-msgs -y

# ROSFMT
git clone https://github.com/xqms/rosfmt.git
rosdep install --from-paths src --ignore-src -r -y

### Python
pip install jupyter matplotlib pandas ipympl shapely scikit-learnm tqdm libpysal esda pyqtdarktheme jupyter-matplotlib colorama


# ALFA AWUS036ACH (RTL8812AU)
mkdir ~/install
cd ~/install
git clone https://github.com/morrownr/8812au-20210820
cd 8812au-20210820
sudo sh ./install-driver.sh

## ohmyzsh
sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

## powerlink10k
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k

## fzf (fuzzy search for history)
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install

## autosuggestions
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions

# install ROS
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install ros-noetic-desktop-full

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool
sudo rosdep init
rosdep update

pip install --upgrade pip
sudo pip install --upgrade pip
sudo pip3 install -U catkin_tools jupyterlab 
pip3 install pyserial # rtk_base

## setup catkin workspace
mkdir -p catkin_ws/src
cd ~/catkin_ws
catkin build

# Setup codebase
cd ~/catkin_ws/src
git clone --recursive https://github.com/robomechanics/environmental_robot.git
```

# HEBI
pip install hebi-py


# Rover Robotics Pro
> Sticking to old code since battery status is not reported in new code. Might need to update robot firmware which requires another dev board.
 
```bash
sudo usermod -aG dialout,sudo,input $USER

## systemctl services
cd setup
sudo cp env.sh /etc/roverrobotics/env.sh
sudo cp roscore.service /etc/systemd/system/roscore.service
sudo cp roverrobotics.service /etc/systemd/system/roverrobotics.service
sudo cp roverrobotics /usr/sbin/roverrobotics
sudo chmod +x /usr/sbin/roverrobotics

## udev rules
sudo 55-roverrobotics.rules /etc/udev/rules.d/55-roverrobotics.rules
sudo udevadm control --reload-rules && sudo udevadm trigger

sudo systemctl enable roscore.service
sudo systemctl enable roverrobotics.service
```

## RTK
git clone --recursive https://github.com/AlexisTM/rtk_ros

## Setup Manual IP for HEBI motors
ROBOT IP, Netmask, Gateway: 192.168.0.200, 24, 192.168.0.1
HEBI IP : 192.168.0.102

## GPS
sudo apt install ros-noetic-microstrain-inertial-driver

## GUI
### Python packages
- PyQtGraph
- pyqt6
- scipy
- pandas
- scikit-learn
- shapely
- wget
- matplotlib

# Notes
- USB INFO
  - RoverRobotics
    > /dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_Rover_Pro-if00-port0 Bus 003 Device 010: ID 0403:6015 Future Technology Devices International, Ltd Bridge(I2C/SPI/UART/FIFO)
  - External Wifi Adapter with Antenna
    > Bus 003 Device 012: ID 0bda:8812 Realtek Semiconductor Corp. RTL8812AU 802.11a/b/g/n/ac 2T2R DB WLAN Adapter
- systemctl services
    ```bash
    sudo systemctl start roscore.service
    sudo systemctl start roverrobotics.service
    ```
- Intel NUC Model: NUC13ANKi7
