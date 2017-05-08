#!/bin/bash
#
# DO NOT RUN DIRECTLY.
# See README.md

THIS_DIR=$(dirname "$BASH_SOURCE")
TMP_SYSTEM_DIR=$(readlink -f ${THIS_DIR}/..)
unset THIS_DIR


# includes
cd "$THIS_DIR"
source "util/helpers.bash"


printf " ###########################################\n"
printf "  UCHILE ROBOT WORKSPACE INSTALLER \n"
printf " ###########################################\n"

## ======================================================
## PRE CHECKS
## ======================================================
printf "\n"

if [ -z "$WS_FOR_ROBOT" ]; then
	printf "Sorry, the WS_FOR_ROBOT env var is not set.\n"
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
if ! _uch_check_rosindigo; then
    printf "Please, install ROS Indigo (ros-indigo-ros-base) before proceeding.\n"
    exit 1
fi

## ======================================================
## FRAMEWORK LOCATION
## ======================================================
printf "\n"

# select a folder were to install the framework (default: /home/user/<robot>_ws)
printf " ------- Framework Location: -------\n"
# comment for debug
_uch_ask_framework_path "$HOME/${robot_name}_ws"
# uncomment for debug
# framework_path=$HOME/DEBUG
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
printf " ============ Workspace Layout Generation: ============ \n"
printf "\n"

## workspace layout
## ---------------------------------------------

mkdir -p "$framework_path"/ros
mkdir -p "$framework_path"/misc
mkdir -p "$framework_path"/deps


## ROS workspaces and overlays
## ---------------------------------------------

# ROS baseline 
source /opt/ros/indigo/setup.bash

# # forks_ws overlays ROS baseline
_uch_create_ws "$framework_path"/ros/forks_ws

# base_ws overlays forks_ws
_uch_create_ws "$framework_path"/ros/base_ws

# soft_ws overlays base_ws
_uch_create_ws "$framework_path"/ros/soft_ws

# high_ws overlays soft_ws
_uch_create_ws "$framework_path"/ros/high_ws



## ======================================================
## <ROBOT>.SH
## ======================================================
printf "\n"
printf "\n"
printf " ============ Setting up ${WS_FOR_ROBOT}.sh ============ \n"

# <robot>.sh
# -----------------------------
printf " - setting up file: $HOME/${WS_FOR_ROBOT}.sh\n"
template="$TMP_SYSTEM_DIR"/templates/robot.sh
cp -f "$template" "$HOME"/"${WS_FOR_ROBOT}".sh
unset template

# replace tags
sed -i "s/<ROBOT_LOWER>/${WS_FOR_ROBOT}/" "$HOME"/"${WS_FOR_ROBOT}".sh
sed -i "s'<FRAMEWORK_PATH>'${framework_path}'" "$HOME"/"${WS_FOR_ROBOT}".sh
printf " - file $HOME/${WS_FOR_ROBOT}.sh is ready...\n\n"

# END
# ----------------------------
export framework_path
printf "\n###########################################\n"
printf " WORKSPACE LAYOUT GENERATION FINISHED! \n"
printf "###########################################\n"
