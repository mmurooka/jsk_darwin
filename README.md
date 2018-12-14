# jsk_darwin

### Install

```bash
# add ros to apt source.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

sudo apt-get update

# install fundamental ros requisities
sudo apt-get install python-catkin-tools python-wstool python-rosdep python-pip python-setuptools ros-indigo-rosbash

source /opt/ros/indigo/setup.bash

# create catkin source directory
mkdir -p ~/ros/ws_darwin/src
cd ~/ros/ws_darwin/src
wstool init
wstool merge  https://raw.githubusercontent.com/mmurooka/jsk_darwin/fix-for-using-robotis-official-repo/darwin-op2.rosinstall -t .
wstool up -j 10

# install dependencies
sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -r -n -y --rosdistro indigo

# build packages
cd ~/ros/ws_darwin
catkin init
catkin build

# source to use programs
echo 'source $HOME/ros/ws_darwin/devel/setup.bash' >> ~/.bashrc
exec -l $SHELL
```

### Launch robot

```bash
ssh robotis@<ip address of darwin> # Darwin IP
sudo bash
roslaunch jsk_darwin darwin_op2.launch
# Servo becomes on and Darwin stands up and then sit down.
```

### Launch robot camera

```bash
ssh robotis@<ip address of darwin> # Darwin IP
sudo bash
roslaunch jsk_darwin darwin_op2_camera.launch
# /image_raw topic starts to be published.
# If program causes error, you need to reboot Darwin PC.
```

### Visualize

```bash
rossetrobot <ip address of darwin>
rossetip # use same network of robot
source ~/ros/ws_darwin/devel/setup.bash
roslaunch jsk_darwin darwin_op2_rviz.launch
# Rviz launches and Darwin model is visualized.
```

### Move from EusLisp

```bash
sudo apt-get install ros-indigo-euslisp ros-indigo-jskeus ros-indigo-roseus ros-indigo-pr2eus
roscd robotis_example/euslisp
roseus robotis_op2-interface.l # run euslisp with prompt $ and ;; for comments

$ init     ;; create irtview the robot model *robot*
$ demo     ;; show the states using itimer
$ demo-stop ;; stop the itimer
$ demo1    ;; show the states using do-until-key
```

![Screen Shot](./Screenshot from 2016-05-08 06:19:25.png)

