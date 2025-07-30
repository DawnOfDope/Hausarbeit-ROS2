#!/bin/bash
# locale  # Sprachabhängigkeiten der Benutzerumgebung nach UTF-8 prüfen

sudo apt update && sudo apt install locales
    
sudo locale-gen en_US en_US.UTF-8

sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

locale  # Einstellungen überprüfen 

#Repos
sudo apt install software-properties-common
sudo add-apt-repository universe
#Key
sudo apt update && sudo apt install curl

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


sudo apt update

sudo apt upgrade
sudo apt install git
sudo apt install ros-humble-desktop
# Skript sourcen
source /opt/ros/humble/setup.bash
sudo echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
 
sudo apt install python3-colcon-common-extensions 
sudo echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
 
 
sudo apt-get install ros-humble-gazebo-*
sudo apt-get install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-turtlebot3*
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup


# mkdir -p ~/turtlebot3_ws/src
# cd ~/turtlebot3_ws/src/
# git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
# git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
# git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
# git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# cd ~/turtlebot3_ws
# colcon build --symlink-install
# echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.bashrc
# echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
# source ~/.bashrc



