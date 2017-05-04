#!/bin/bash
#
# DO NOT RUN DIRECTLY.
# See README.md


printf " ###########################################\n"
printf "  UCHILE ROBOT WORKSPACE INSTALLER \n"
printf " ###########################################\n"

## ======================================================
## UTILITIES
## ======================================================

cd "$(dirname "$BASH_SOURCE")"
source "util/helpers.bash"

## ======================================================
## PRE CHECKS
## ======================================================
printf "\n"

if [ -z "$WS_FOR_ROBOT" ]; then
	printf "Sorry, the WS_FOR_ROBOT env var is not defined.\n"
	printf "Maybe you are not running this script as required. See the README.md file.\n"
    exit 1
fi

if [[ "$WS_FOR_ROBOT" =~ [^a-z0-9_] ]]; then
	printf "Sorry, the WS_FOR_ROBOT env var must only contain lowercase [a-z] characters.\n"
	printf "Found: %s\n" "$WS_FOR_ROBOT"
    exit 1
fi
robot_name="$WS_FOR_ROBOT"
printf " - Robot name: %s\n" "$robot_name"

# ROS is installed
if ! _uch_installer_check_rosindigo; then
    printf "Please, install ROS Indigo (ros-indigo-ros-base) before proceeding.\n"
    exit 1
fi

## ======================================================
## FRAMEWORK LOCATION
## ======================================================
printf "\n"

# select a folder were to install the framework (default: /home/user/bender_ws)
printf " ------- Framework Location: -------\n"
_uch_installer_ask_framework_path "$HOME/${robot_name}_ws"
printf " - Using path: %s\n" "$framework_path"


## ======================================================
## PREPARE WORKSPACE
## ======================================================
# at this point:
# - framework_path contains the fullpath to a (existent) folder
# - framework_path should be a non empty directory
# - ROS indigo is installed under /opt/ros/indigo
#
printf "\n"
printf "\n"
printf " ============ Workspace Preparation: ============ \n"
printf "\n"

## workspace layout
## ---------------------------------------------

mkdir -p "$framework_path"/ros
mkdir -p "$framework_path"/misc
mkdir -p "$framework_path"/install


## ROS workspaces and overlays
## ---------------------------------------------

# ROS baseline 
source /opt/ros/indigo/setup.bash

# forks_ws overlays ROS baseline
_uch_installer_reset_ws "$framework_path"/ros/forks_ws

# base_ws overlays forks_ws
_uch_installer_reset_ws "$framework_path"/ros/base_ws

# soft_ws overlays base_ws
_uch_installer_reset_ws "$framework_path"/ros/soft_ws

# high_ws overlays soft_ws
_uch_installer_reset_ws "$framework_path"/ros/high_ws

unset framework_path

# END
# ----------------------------
printf "###########################################\n"
printf " DONE! \n"
printf "###########################################\n"
