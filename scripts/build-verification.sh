#!/bin/bash

#####################################################################
# ROS 2 Persistent Parameter Server
#
# This script builds parameter server within ros docker images.
#
# To avoid updating and modifying the files under `.github/workflows`,
# this scripts should be adjusted building process accordingly.
# And `.github/workflows` just calls this script in the workflow pipeline.
# This allows us to maintain the workflow process easier for contributors.
#
#####################################################################

########################
# Function Definitions #
########################

function mark {
    export $1=`pwd`;
}

function exit_trap() {
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function install_prerequisites () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: update and install dependent packages."
    apt update && apt upgrade -y
    apt install -y ros-${ROS_DISTRO}-desktop ros-${ROS_DISTRO}-rmw-cyclonedds-cpp --no-install-recommends
    apt install -y libyaml-cpp-dev libboost-program-options-dev libboost-filesystem-dev
    cd $there
}

function setup_build_colcon_env () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: set up colcon build environement."
    mkdir -p ${COLCON_WORKSPACE}/src
    cd ${COLCON_WORKSPACE}
    cp -rf $there ${COLCON_WORKSPACE}/src
}

function build_parameter_server () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: build ROS 2 parameter server."
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${COLCON_WORKSPACE}
    colcon build --symlink-install --packages-select parameter_server ros2_persistent_parameter_server_test
}

function test_parameter_server () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: test ROS 2 parameter server."
    source /opt/ros/${ROS_DISTRO}/setup.bash
    cd ${COLCON_WORKSPACE}

    # TODO(@fujitatomoya): currently unit tests are missing for parameter server with `colcon test`.

    # source the parameter server local packages
    source ./install/local_setup.bash
    # setup and execute the system test
    cp ./src/ros2/ros2_persist_parameter_server/server/param/parameter_server.yaml /tmp/test
    ./src/ros2_persist_parameter_server/test/test.py
}

########
# Main #
########

export DEBIAN_FRONTEND=noninteractive
export COLCON_WORKSPACE=/tmp/colcon_ws

# mark the working space root directory, so that we can come back anytime with `cd $there`
mark there

# set the trap on error
trap exit_trap ERR

# call install functions in sequence
install_prerequisites
setup_build_colcon_env
build_parameter_server
test_parameter_server

exit 0
