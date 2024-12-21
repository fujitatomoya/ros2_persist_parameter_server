#!/bin/bash

#################################################################################
# This script builds and releases ros2 persistent parameter server docker images.
#################################################################################

################
# User Setting #
################

DOCKERHUB_USERNAME="${DOCKERHUB_USERNAME:-tomoyafujita}"
COLCON_WS="${COLCON_WS:-/root/colcon_ws}"

ros_distros=(
    "humble"
    "jazzy"
    "rolling"
)

######################
# Options (defaults) #
######################

build_image=false
upload_image=false

########################
# Function Definitions #
########################

function print_usage() {
    echo "Usage: $0 [-b] [-u]"
    echo "Options(default):"
    echo "  -b : build docker container images (default: false)"
    echo "  -u : upload images to DockerHub (default: false)"
    exit 1
}

function exit_trap() {
    # shellcheck disable=SC2317  # Don't warn about unreachable commands in this function
    if [ $? != 0 ]; then
        echo "Command [$BASH_COMMAND] is failed"
        exit 1
    fi
}

function check_dockerhub_setting () {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: checking dockerhub setting and configuration."
    if [ -z "$DOCKERHUB_USERNAME" ]; then
        echo "DOCKERHUB_USERNAME is not set."
        exit 1
    fi
    # check if docker login succeeds
    docker login
}

function command_exist() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: checking $1 command exists."
    if command -v "$1" >/dev/null 2>&1; then
        echo "$1 exists."
    else
        echo "Error: $1 not found."
        exit 1
    fi
}

function build_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: building ros2 persistent parameter server docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro image building"
        docker build --rm -f ./docker/Dockerfile --build-arg="ROS_DISTRO=$distro" --build-arg="COLCON_WS=$COLCON_WS" -t $DOCKERHUB_USERNAME/ros2_param_server:$distro .
    done
    echo "----- all images successfully generated!!! -----"
}

function upload_images() {
    trap exit_trap ERR
    echo "[${FUNCNAME[0]}]: uploading ros2 persistent parameter server docker container images."
    for distro in "${ros_distros[@]}"; do
        echo "----- $distro image uploading"
        # TODO@fujitatomoya: support multi-arch docker images
        docker push $DOCKERHUB_USERNAME/ros2_param_server:$distro
    done
    echo "----- all images successfully verified!!! -----"
}

########
# Main #
########

# set the trap on error
trap exit_trap ERR

# parse command line options
while getopts ":bvu" opt; do
    case $opt in
        b)
            build_image=true
            ;;
        u)
            upload_image=true
            ;;
        \?)
            echo "Invalid option: -$OPTARG"
            print_usage
            ;;
    esac
done
shift $((OPTIND-1))

# check settings
command_exist docker
check_dockerhub_setting

# building images
if [ "$build_image" = true ]; then
    build_images
fi

# upload images
if [ "$upload_image" = true ]; then
    upload_images
fi

exit 0
