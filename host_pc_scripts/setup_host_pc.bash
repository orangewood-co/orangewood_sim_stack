#!/usr/bin/env bash

#Install ROS Noetic
chmod +x ros_install_noetic.sh
./ros_install_noetic.sh

mkdir -p ~/orangewood_ws/src

cd ~/orangewood_ws/src

git clone https://github.com/orangewood-co/orangewood_sim_stack.git

cd ~/orangewood_ws

rosdep install --from-paths src --ignore-src -r -y

catkin_make

echo "source ~/orangewood_ws/devel/setup.bash" >> /home/$USER/.bashrc

source /home/$USER/.bashrc

