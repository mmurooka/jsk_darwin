#!/bin/bash

set -e

function travis_time_start {
    set +x
    TRAVIS_START_TIME=$(date +%s%N)
    TRAVIS_TIME_ID=$(cat /dev/urandom | tr -dc 'a-z0-9' | fold -w 8 | head -n 1)
    TRAVIS_FOLD_NAME=$1
    echo -e "\e[0Ktraivs_fold:start:$TRAVIS_FOLD_NAME"
    echo -e "\e[0Ktraivs_time:start:$TRAVIS_TIME_ID"
    set -x
}
function travis_time_end {
    set +x
    _COLOR=${1:-32}
    TRAVIS_END_TIME=$(date +%s%N)
    TIME_ELAPSED_SECONDS=$(( ($TRAVIS_END_TIME - $TRAVIS_START_TIME)/1000000000 ))
    echo -e "traivs_time:end:$TRAVIS_TIME_ID:start=$TRAVIS_START_TIME,finish=$TRAVIS_END_TIME,duration=$(($TRAVIS_END_TIME - $TRAVIS_START_TIME))\n\e[0K"
    echo -e "traivs_fold:end:$TRAVIS_FOLD_NAME"
    echo -e "\e[0K\e[${_COLOR}mFunction $TRAVIS_FOLD_NAME takes $(( $TIME_ELAPSED_SECONDS / 60 )) min $(( $TIME_ELAPSED_SECONDS % 60 )) sec\e[0m"
    set -x
}

function travis_wait {
  set +x
  local timeout=$1

  if [[ $timeout =~ ^[0-9]+$ ]]; then
    # looks like an integer, so we assume it's a timeout
    shift
  else
    # default value
    timeout=20
  fi

  local cmd="$@"
  local log_file=travis_wait_$$.log

  $cmd &>$log_file &
  local cmd_pid=$!

  travis_jigger $! $timeout $cmd &
  local jigger_pid=$!
  local result

  {
    wait $cmd_pid 2>/dev/null
    result=$?
    ps -p$jigger_pid &>/dev/null && kill $jigger_pid
  }

  if [ $result -eq 0 ]; then
    echo -e "\n${ANSI_GREEN}The command $cmd exited with $result.${ANSI_RESET}"
  else
    echo -e "\n${ANSI_RED}The command $cmd exited with $result.${ANSI_RESET}"
  fi

  echo -e "\n${ANSI_GREEN}Log:${ANSI_RESET}\n"
  cat $log_file

  set -x
  return $result
}

function travis_jigger {
  # helper method for travis_wait()
  local cmd_pid=$1
  shift
  local timeout=$1 # in minutes
  shift
  local count=0

  # clear the line
  echo -e "\n"

  while [ $count -lt $timeout ]; do
    count=$(($count + 1))
    echo -ne "Still running ($count of $timeout): $@\r"
    sleep 60
  done

  echo -e "\n${ANSI_RED}Timeout (${timeout} minutes) reached. Terminating \"$@\"${ANSI_RESET}\n"
  kill -9 $cmd_pid
}

travis_time_start setup.apt
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get -q -qq update
sudo apt-get -q -qq -y install rsync unzip
sudo apt-get -q -qq -y install ros-$ROS_DISTRO-catkin ros-$ROS_DISTRO-ros
sudo easy_install pip
sudo -H pip install -U pip setuptools
sudo -H pip install -U numpy
sudo -H pip install -U catkin_pkg catkin_tools rosdep wstool
travis_time_end

travis_time_start rosdep.update
if [ ! -e "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
  sudo rosdep init
fi
ret=1; rosdep update || while [ $ret != 0 ]; do sleep 1; rosdep update && ret=0 || echo "failed"; done
travis_time_end

travis_time_start rosdep.install
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
wstool init
wstool merge $CI_SOURCE_PATH/darwin-op2.rosinstall
wstool up -t ~/catkin_ws/src -j10
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -r -n -y # --skip-keys=opencv3
travis_time_end

travis_time_start catkin.build
set +x
source /opt/ros/$ROS_DISTRO/setup.bash
set -x
cd ~/catkin_ws
catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
travis_wait 60 catkin build --summarize --no-status $BUILD_PKGS -p2 --make-args -j2 -- --cmake-args "$CMAKE_ARGS" --
travis_time_end

travis_time_start catkin.run_tests
set +x
source devel/setup.bash
set -x
travis_wait 60 catkin run_tests -i --no-deps --no-status $TEST_PKGS -p1 --make-args -j1 -- --cmake-args "$CMAKE_ARGS" --
travis_time_end

catkin_test_results --verbose --all build
