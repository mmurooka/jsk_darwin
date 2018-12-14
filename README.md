# jsk_darwin

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
catkin build op2_manager robotis_op2_description jsk_darwin

# source to use programs
echo 'source $HOME/ros/ws_darwin/devel/setup.bash' >> ~/.bashrc
exec -l $SHELL
```

### Launch example

- display imu value

```bash

# imu transform
sudo apt-get install ros-indigo-hector-imu-attitude-to-tf
sudo apt-get install ros-indigo-rviz-imu-plugin

sudo bash
# servo on
roslaunch robotis_example robotis_example.launch
# another sudo bash
roslaunch robotis_example imu_view.launch

# check imu value
rostopic echo /imu
```

- visualization

```bash
source ~/ros/ws_darwin/devel/setup.bash
rossetrobot <ip address of darwin>
rossetip # use same network of robot

roslaunch robotis_op2_description robotis_op2_rviz.launch
# you can set /map as Fixed Frame to see robot pose.
```

- Robot camera view

```bash
sudo bash

roslaunch robotis_op2_camera robotis_op2_camera.launch
# you can view with /image_raw on rviz or rqt_image_view.
```

- Ball track view

```bash
sudo bash

roslaunch ball_detector ball_detector_from_op.launch
# you can view the result with /ball_detector_node/image_out on rviz or rqt_image_view.
```

- euslisp example

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

