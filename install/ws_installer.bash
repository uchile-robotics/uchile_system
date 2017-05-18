#!/bin/bash
#
# For complete installation instructions see README.md 
#
# tl;dr: 
# 1.- Install ROS Indigo
# 2.- Unsource ROS
# 3.- Clone and run this script
#     $ chmod +x ws_installer.bash
#     $ ./ws_installer.bash
#

## ======================================================
## HEADERS
## ======================================================
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
## PARAMS
## ======================================================
ask_framework_path=false

force_rebuild=false # TODO

## ======================================================
## PRE CHECKS
## ======================================================
printf "\n"

# ROS is installed
if ! _uchile_check_rosindigo; then
    printf "\n[FAIL] Please, install ROS Indigo (ros-indigo-ros-base) before proceeding.\n"
    exit 1
fi

# ROS is not sourced
if [ ! -z "$ROS_DISTRO" ]; then
    printf "\n[FAIL] ROS must not be sourced on this shell!. Found ROS_DISTRO=$ROS_DISTRO\n"
    exit 1
fi


## ======================================================
## FRAMEWORK LOCATION
## ======================================================
printf "\n"

# select a folder were to install the framework (default: /home/user/uchile_ws)
printf " ------- Framework Location: -------\n"
framework_path="$HOME"/uchile_ws
if $ask_framework_path; then
	_uchile_ask_framework_path "$HOME/uchile_ws"
fi
unset ask_framework_path
mkdir -p $framework_path
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
printf " ============ Workspace Layout Generation: ============ \n"
printf "\n"

## workspace layout
## ---------------------------------------------

# miscellaneous
mkdir -p "$framework_path"/misc
mkdir -p "$framework_path"/misc/graveyard

# for dependencies installation
mkdir -p "$framework_path"/deps
mkdir -p "$framework_path"/deps/maqui
mkdir -p "$framework_path"/deps/bender
mkdir -p "$framework_path"/deps/common

# all ROS packages
mkdir -p "$framework_path"/pkgs
mkdir -p "$framework_path"/pkgs/base_ws
mkdir -p "$framework_path"/pkgs/soft_ws
mkdir -p "$framework_path"/pkgs/high_ws

# available ROS workspaces
mkdir -p "$framework_path"/ros
mkdir -p "$framework_path"/ros/bender_ws
mkdir -p "$framework_path"/ros/maqui_ws
mkdir -p "$framework_path"/ros/all_ws


## ROS workspaces and overlays
## ---------------------------------------------

# bender_ws
bash -c "source $TMP_SYSTEM_DIR/install/util/helpers.bash; _uchile_create_complete_ws $framework_path/ros/bender_ws"

# maqui_ws
bash -c "source $TMP_SYSTEM_DIR/install/util/helpers.bash; _uchile_create_complete_ws $framework_path/ros/maqui_ws"

# all_ws
bash -c "source $TMP_SYSTEM_DIR/install/util/helpers.bash; _uchile_create_complete_ws $framework_path/ros/all_ws"


## ======================================================
## UCHILE.SH
## ======================================================
printf "\n"
printf " ============ Setting up uchile.sh ============ \n"

# uchile.sh
# -----------------------------
printf " - setting up file: $HOME/uchile.sh\n"
template="$TMP_SYSTEM_DIR"/templates/robot.sh
cp -f "$template" "$HOME"/uchile.sh
unset template

# replace tags
sed -i "s'<FRAMEWORK_PATH>'${framework_path}'" "$HOME"/uchile.sh
printf " - file $HOME/uchile.sh is ready...\n\n"


## ======================================================
## REPOSITORIES
## ======================================================

printf "\n\n ============ Retrieving Repositories ============ \n"
cd "$framework_path"

# 15 min cache
credential_helper=$(git config credential.helper)
if [ $? -eq 0 ]; then
	printf " - A global git config credential.helper configuration was found:\n"
	printf "   ... '$ git config credential.helper' returned '%s'\n" "$credential_helper"
else
	printf " - A global git config credential.helper configuration was not found.\n"
	printf " - Setting up a 15 min git credential cache:\n"
	printf "   ... $ git config --global credential.helper 'cache --timeout=900'\n"
	printf "   ... It is recommended to set a larger cache, e.g. 1 day: timeout=86400\n"
	git config --global credential.helper 'cache --timeout=900'
fi

# where to copy the git hook from
_hook_template="$TMP_SYSTEM_DIR"/hooks/hooks/pre-commit


## MISC
## ----------------------------------------------------------------------------

# wiki
_uchile_get_repository "misc/wiki" "https://github.com/uchile-robotics/uchile_system.wiki.git"

# graveyard
_uchile_get_repository "misc/graveyard/code_graveyard"    "https://github.com/uchile-robotics-graveyard/code_graveyard"
_uchile_get_repository "misc/graveyard/uchile_embedded"   "https://github.com/uchile-robotics-graveyard/uchile_embedded"
_uchile_get_repository "misc/graveyard/old_page"          "https://github.com/uchile-robotics-graveyard/bender_page"
_uchile_get_repository "misc/graveyard/BenderCode"        "https://github.com/uchile-robotics/BenderCode"
_uchile_get_repository "misc/graveyard/BenderCode_Indigo" "https://github.com/uchile-robotics/BenderCode_Indigo"

# page
_uchile_get_repository "misc/webpage" "https://github.com/uchile-robotics/uchile-robotics.github.io"


## system
## ----------------------------------------------------------------------------

# system
_uchile_get_repository "system" "https://github.com/uchile-robotics/uchile_system"
_uchile_enable_githook "system" "$_hook_template"


## layers
## ----------------------------------------------------------------------------

_uchile_get_repository "pkgs/base_ws/uchile_common"       "https://github.com/uchile-robotics/uchile_common"
_uchile_get_repository "pkgs/base_ws/uchile_knowledge"    "https://github.com/uchile-robotics/uchile_knowledge"
_uchile_get_repository "pkgs/base_ws/uchile_tools"        "https://github.com/uchile-robotics/uchile_tools"
_uchile_get_repository "pkgs/base_ws/bender_core"         "https://github.com/uchile-robotics/bender_core"

_uchile_get_repository "pkgs/soft_ws/uchile_hri"          "https://github.com/uchile-robotics/uchile_hri"
_uchile_get_repository "pkgs/soft_ws/uchile_navigation"   "https://github.com/uchile-robotics/uchile_navigation"
_uchile_get_repository "pkgs/soft_ws/uchile_manipulation" "https://github.com/uchile-robotics/uchile_manipulation"
_uchile_get_repository "pkgs/soft_ws/uchile_perception"   "https://github.com/uchile-robotics/uchile_perception"

_uchile_get_repository "pkgs/high_ws/uchile_high"         "https://github.com/uchile-robotics/uchile_high"
_uchile_get_repository "pkgs/high_ws/maqui_bringup"       "https://github.com/uchile-robotics/maqui_bringup"
_uchile_get_repository "pkgs/high_ws/bender_bringup"      "https://github.com/uchile-robotics/bender_bringup"

_uchile_enable_githook "pkgs/base_ws/uchile_common"       "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/uchile_knowledge"    "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/uchile_tools"        "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/bender_core"         "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_hri"          "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_navigation"   "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_manipulation" "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_perception"   "$_hook_template"
_uchile_enable_githook "pkgs/high_ws/uchile_high"         "$_hook_template"
_uchile_enable_githook "pkgs/high_ws/maqui_bringup"       "$_hook_template"
_uchile_enable_githook "pkgs/high_ws/bender_bringup"      "$_hook_template"


## forks
## ----------------------------------------------------------------------------

# UNKNOWN!
# https://github.com/uchile-robotics-forks/console_bridge
# https://github.com/uchile-robotics-forks/robot_model
# https://github.com/uchile-robotics-forks/libqi-release
# https://github.com/uchile-robotics-forks/libqicore-release
# https://github.com/uchile-robotics-forks/nao_robot
# https://github.com/uchile-robotics-forks/naoqi_driver
# https://github.com/uchile-robotics-forks/naoqi_bridge
# https://github.com/uchile-robotics-forks/naoqi_bridge_msgs
# https://github.com/uchile-robotics-forks/naoqi_dcm_driver
# https://github.com/uchile-robotics-forks/pepper_dcm_robot
# https://github.com/uchile-robotics-forks/pepper_virtual
# https://github.com/uchile-robotics-forks/pepper_robot
# https://github.com/uchile-robotics-forks/pepper_meshes
# https://github.com/uchile-robotics-forks/pepper_moveit_config

# forks: rosaria
_uchile_get_repository "pkgs/forks_ws/rosaria" "https://github.com/uchile-robotics-forks/rosaria" "master"

# fork: dynamixel_motor
_uchile_get_repository "pkgs/forks_ws/dynamixel_motor" "https://github.com/uchile-robotics-forks/dynamixel_motor" "develop"

# fork: usb_cam
_uchile_get_repository "pkgs/forks_ws/usb_cam" "https://github.com/uchile-robotics-forks/usb_cam" "0.3.4"

# fork: urg_node
_uchile_get_repository "pkgs/forks_ws/urg_node" "https://github.com/uchile-robotics-forks/urg_node" "0.1.9"

# fork: navigation
_uchile_get_repository "pkgs/forks_ws/navigation" "https://github.com/uchile-robotics-forks/navigation" "kinetic-devel"

# fork: open_ptrack
_uchile_get_repository "pkgs/forks_ws/open_ptrack" "https://github.com/uchile-robotics-forks/open_ptrack" "master"


## deps
## ----------------------------------------------------------------------------

# # install python-aiml
# if [ ! -d "$framework_path"/deps/base/knowledge/python-aiml ]; then
# 	mkdir -p "$framework_path"/deps/base/knowledge/
# 	cd "$framework_path"/deps/base/knowledge/
# 	git clone https://github.com/uchile-robotics-forks/python-aiml
# 	cd python-aiml
# 	sudo python setup.py install
# 	cd "$framework_path"
# fi

unset _hook_template



# END
# ----------------------------
printf "\n"
printf "The installation is almost ready!. Just follow the installation\n"
printf "instructions on the system README.md file.\n"
printf "\n"

unset framework_path

# END
# ----------------------------
printf "\n###########################################\n"
printf " WORKSPACE GENERATION FINISHED! \n"
printf "###########################################\n"
