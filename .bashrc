source /opt/ros/kinetic/setup.bash
source ~/catkin_ca/devel/setup.bash

alias gb='gedit ~/.bashrc'
alias sb='source ~/.bashrc'
alias cm='cd ~/catkin_ca && catkin_make'
alias tm='cd ~/ctl_ws && catkin_make'
alias gz='roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch'
alias killgz='killall gzserver & killall gzclient'
alias plot='rosrun plotjuggler PlotJuggler'
alias urdf='nautilus /opt/ros/kinetic/share/turtlebot3_description'

#source ~/ctl_ws/devel/setup.bash


export ROS_MASTER_URL=http://localhost:11311
export ROS_HOSTNAME=localhost

export TURTLEBOT3_MODEL=burger
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/catkin_ca/devel/lib
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/.gazebo/models
