# System Packages
```bash
# Install ROS
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

## Setup catkin workspace
mkdir -p catkin_ws/src
cd ~/catkin_ws
catkin build

## Setup codebase
cd ~/catkin_ws/src
git clone --recursive https://github.com/robomechanics/environmental_robot.git

# Basic dependencies
sudo apt install zsh tmux autojump git curl build-essential python3-pip -y

# ROS Packages
sudo apt-get install net-tools openssh-server python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential ros-noetic-serial ros-noetic-joy ros-noetic-twist-mux ros-noetic-tf2-geometry-msgs ros-noetic-robot-localization ros-noetic-gmapping libqt5websockets5-dev ros-noetic-rtcm-msgs ros-noetic-rosmon ros-noetic-usb-cam ros-noetic-rtcm_msgs ros-noetic-nmea_msgs ros-noetic-mavros ros-noetic-microstrain-inertial-driver
ros-noetic-move-base ros-noetic-move-base-msgs -y

# ROSFMT
git clone https://github.com/xqms/rosfmt.git
rosdep install --from-paths src --ignore-src -r -y

# Python
pip install jupyter matplotlib pandas ipympl shapely scikit-learn tqdm libpysal esda pyqtdarktheme jupyter-matplotlib colorama scipy scipy PyQtGraph wget matplotlib

# HEBI
pip install hebi-py

# Terminal
## ohmyzsh
sh -c "$(wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

## powerlink10k
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k

## fzf (fuzzy search for history)
git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
~/.fzf/install

## autosuggestions
git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions
```

## RTK
```bash
git clone --recursive https://github.com/AlexisTM/rtk_ros
```

