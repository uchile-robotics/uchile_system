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
TMP_SYSTEM_DIR=$(readlink -f "${THIS_DIR}/..")
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
    printf "\n[FAIL] ROS must not be sourced on this shell!. Found ROS_DISTRO=%s\n" "$ROS_DISTRO"
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
mkdir -p "$framework_path"
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
mkdir -p "$framework_path"/ros/bender
mkdir -p "$framework_path"/ros/maqui
mkdir -p "$framework_path"/ros/all


## ROS workspaces and overlays
## ---------------------------------------------

# bender
bash -c "source $TMP_SYSTEM_DIR/install/util/helpers.bash; _uchile_create_complete_ws $framework_path/ros/bender"

# maqui
bash -c "source $TMP_SYSTEM_DIR/install/util/helpers.bash; _uchile_create_complete_ws $framework_path/ros/maqui"

# all
bash -c "source $TMP_SYSTEM_DIR/install/util/helpers.bash; _uchile_create_complete_ws $framework_path/ros/all"


## ======================================================
## UCHILE.SH
## ======================================================
printf "\n"
printf " ============ Setting up uchile.sh ============ \n"

# uchile.sh
# -----------------------------
printf " - setting up file: %s\n" "$HOME/uchile.sh"
template="$TMP_SYSTEM_DIR"/templates/robot.sh
cp -f "$template" "$HOME"/uchile.sh
unset template

# replace tags
sed -i "s'<FRAMEWORK_PATH>'${framework_path}'" "$HOME"/uchile.sh
printf " - file %s is ready...\n\n" "$HOME/uchile.sh"


## ======================================================
## REPOSITORIES
## ======================================================

printf "\n\n ============ Retrieving Repositories ============ \n"
cd "$framework_path"

# 15 min cache
_unset_helper=false
_credential_helper=$(git config credential.helper)
if [ $? -eq 0 ]; then
	printf " - A global git config credential.helper configuration was found:\n"
	printf "   ... '$ git config credential.helper' returned '%s'\n" "$_credential_helper"
else
	printf " - A global git config credential.helper configuration was not found.\n"
	printf " - Setting up a 15 min git credential cache:\n"
	printf "   ... $ git config --global credential.helper 'cache --timeout=900'\n"
	printf "   ... It is recommended to set a larger cache, e.g. 1 day: timeout=86400\n"
	git config --global credential.helper 'cache --timeout=900'
	_unset_helper=true
fi
unset _credential_helper


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


## layers
## ----------------------------------------------------------------------------

_uchile_get_repository "pkgs/base_ws/uchile_common"       "https://github.com/uchile-robotics/uchile_common"
_uchile_get_repository "pkgs/base_ws/uchile_knowledge"    "https://github.com/uchile-robotics/uchile_knowledge"
_uchile_get_repository "pkgs/base_ws/uchile_tools"        "https://github.com/uchile-robotics/uchile_tools"
_uchile_get_repository "pkgs/base_ws/bender_core"         "https://github.com/uchile-robotics/bender_core"
_uchile_get_repository "pkgs/base_ws/maqui_core"          "https://github.com/uchile-robotics/maqui_core"

_uchile_get_repository "pkgs/soft_ws/uchile_hri"          "https://github.com/uchile-robotics/uchile_hri"
_uchile_get_repository "pkgs/soft_ws/uchile_navigation"   "https://github.com/uchile-robotics/uchile_navigation"
_uchile_get_repository "pkgs/soft_ws/uchile_manipulation" "https://github.com/uchile-robotics/uchile_manipulation"
_uchile_get_repository "pkgs/soft_ws/uchile_perception"   "https://github.com/uchile-robotics/uchile_perception"

_uchile_get_repository "pkgs/high_ws/uchile_high"         "https://github.com/uchile-robotics/uchile_high"
_uchile_get_repository "pkgs/high_ws/maqui_bringup"       "https://github.com/uchile-robotics/maqui_bringup"
_uchile_get_repository "pkgs/high_ws/bender_bringup"      "https://github.com/uchile-robotics/bender_bringup"


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


# unset credential helper if needed
if $_unset_helper; then
	git config --global --unset credential.helper
fi
unset _unset_helper


## ======================================================
## GIT HOOKS
## ======================================================

printf "\n\n ============ Installing Git Hooks ============ \n"

# where to copy the git hook from
_hook_template="$TMP_SYSTEM_DIR"/hooks/pre-commit
_uchile_enable_githook "system" "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/uchile_common"       "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/uchile_knowledge"    "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/uchile_tools"        "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/bender_core"         "$_hook_template"
_uchile_enable_githook "pkgs/base_ws/maqui_core"          "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_hri"          "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_navigation"   "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_manipulation" "$_hook_template"
_uchile_enable_githook "pkgs/soft_ws/uchile_perception"   "$_hook_template"
_uchile_enable_githook "pkgs/high_ws/uchile_high"         "$_hook_template"
_uchile_enable_githook "pkgs/high_ws/maqui_bringup"       "$_hook_template"
_uchile_enable_githook "pkgs/high_ws/bender_bringup"      "$_hook_template"

unset _hook_template


## ======================================================
## LINK WORKSPACES
## ======================================================

printf "\n\n ============ Linking Workspaces ============ \n"

function _uchile_link_common_ ()
{
	local target_ws
	target_ws="$1"

	printf "\n - Linking common repositories for %s workspace:\n" "$target_ws"
	
	# forks
	_uchile_link_ "forks_ws/navigation"         "$target_ws/forks_ws/src/navigation"
	_uchile_link_ "forks_ws/open_ptrack"        "$target_ws/forks_ws/src/open_ptrack"

	# base
	_uchile_link_ "base_ws/uchile_common"       "$target_ws/base_ws/src/uchile_common"
	_uchile_link_ "base_ws/uchile_knowledge"    "$target_ws/base_ws/src/uchile_knowledge"
	_uchile_link_ "base_ws/uchile_tools"        "$target_ws/base_ws/src/uchile_tools"

	# soft
	_uchile_link_ "soft_ws/uchile_hri"          "$target_ws/soft_ws/src/uchile_hri"
	_uchile_link_ "soft_ws/uchile_navigation"   "$target_ws/soft_ws/src/uchile_navigation"
	_uchile_link_ "soft_ws/uchile_manipulation" "$target_ws/soft_ws/src/uchile_manipulation"
	_uchile_link_ "soft_ws/uchile_perception"   "$target_ws/soft_ws/src/uchile_perception"

	# high
	_uchile_link_ "high_ws/uchile_high"         "$target_ws/high_ws/src/uchile_high"
}

# COMMON REPOSITORIES
_uchile_link_common_ "bender"
_uchile_link_common_ "maqui"
_uchile_link_common_ "all"


# BENDER ONLY REPOSITORIES
_uchile_link_ "forks_ws/open_ptrack"        "bender/forks_ws/src/open_ptrack"
_uchile_link_ "forks_ws/rosaria"            "bender/forks_ws/src/rosaria"
_uchile_link_ "forks_ws/dynamixel_motor"    "bender/forks_ws/src/dynamixel_motor"
_uchile_link_ "forks_ws/urg_node"           "bender/forks_ws/src/urg_node"
_uchile_link_ "forks_ws/usb_cam"            "bender/forks_ws/src/usb_cam"
_uchile_link_ "base_ws/bender_core"         "bender/base_ws/src/bender_core"
_uchile_link_ "high_ws/bender_bringup"      "bender/high_ws/src/bender_bringup"


# MAQUI ONLY REPOSITORIES
_uchile_link_ "base_ws/maqui_core"          "maqui/base_ws/src/maqui_core"
_uchile_link_ "high_ws/maqui_bringup"       "maqui/high_ws/src/maqui_bringup"


# REMAINING REPOS FOR ALL
_uchile_link_ "forks_ws/open_ptrack"        "all/forks_ws/src/open_ptrack"
_uchile_link_ "forks_ws/rosaria"            "all/forks_ws/src/rosaria"
_uchile_link_ "forks_ws/dynamixel_motor"    "all/forks_ws/src/dynamixel_motor"
_uchile_link_ "forks_ws/urg_node"           "all/forks_ws/src/urg_node"
_uchile_link_ "forks_ws/usb_cam"            "all/forks_ws/src/usb_cam"
_uchile_link_ "base_ws/bender_core"         "all/base_ws/src/bender_core"
_uchile_link_ "base_ws/maqui_core"          "all/base_ws/src/maqui_core"
_uchile_link_ "high_ws/bender_bringup"      "all/high_ws/src/bender_bringup"
_uchile_link_ "high_ws/maqui_bringup"       "all/high_ws/src/maqui_bringup"

unset _uchile_link_
unset _uchile_link_common_


# END
# ----------------------------
printf "\n###########################################\n"
printf " WORKSPACE GENERATION FINISHED! \n"
printf "###########################################\n"
printf "\n"
printf "The installation is almost ready!. Just follow the installation\n"
printf "instructions on the system README.md file.\n"
printf "\n"

unset framework_path
