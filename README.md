# jsk_darwin

Author: Yuki Furuta <furushchev@mail.ru>

### setup ROS on Darwin OP2

```bash
# add ros to apt source.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

sudo apt-get update

# install fundamental ros requisities
sudo apt-get install python-catkin-tools python-wstool python-rosdep python-pip python-setuptools ros-indigo-rosbash

source /opt/ros/indigo/setup.bash

# create catkin source directory
mkdir -p ~/ros/indigo/src
cd ~/ros/indigo/src
wstool init

wstool merge jsk_darwin/darwin-op2.rosinstall -t .
wstool up -j 3

# install dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -n -y

# build packages
cd ~/ros/indigo
catkin init
catkin build

# source to use programs
echo 'source $HOME/ros/indigo/devel/setup.bash' >> ~/.bashrc
exec -l $SHELL
```

### Launch example

```bash
sudo bash

# servo on
roslaunch robotis_example robotis_example.launch

# check imu value
rostopic echo /imu
```

