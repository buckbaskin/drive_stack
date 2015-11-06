echo 'start:'
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
echo 'leys added'
sudo apt-get update
sudo apt-get install ros-indigo-ros-base
echo 'ros added'
sudo rosdep init
sudo rosdep update
source /opt/ros/indigo/setup.bash
echo 'rosdep done'