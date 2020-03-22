#!/bin/bash
#
# For complete installation instructions see README.md 
#
# tl;dr: 
# 1.- Install ROS melodic
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

# includes
cd "$THIS_DIR"
unset THIS_DIR
source "util/helpers.bash"


printf " ###########################################\n"
printf "  UCHILE ROBOT WORKSPACE INSTALLER \n"
printf " ###########################################\n"

## ======================================================
## PARAMS
## ======================================================
ask_framework_path=false

# TODOs
# force_rebuild=false # force workspace rebuilds
# avoid extra=false   # Avoid cloning and setting other 
#                       environments. Just sets up a 
#                       single robot env.

## ======================================================
## PRE CHECKS
## ======================================================
printf "\n"

# ROS is installed
if ! _uchile_check_rosmelodic; then
    printf "\n[FAIL] Please, install ROS melodic (ros-melodic-ros-base) before proceeding.\n"
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
# - ROS melodic is installed under /opt/ros/melodic
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
_uchile_create_complete_ws "${framework_path}/ros/bender"

# maqui
_uchile_create_complete_ws "${framework_path}/ros/maqui"

# all
_uchile_create_complete_ws "${framework_path}/ros/all"


## ======================================================
## UCHILE.SH
## ======================================================
printf "\n"
printf " ============ Setting up uchile.sh ============ \n"

# uchile.sh
# -----------------------------
printf " - setting up file: %s\n" "$HOME/uchile.sh"
template="$TMP_SYSTEM_DIR"/templates/uchile.sh
cp -f "$HOME"/uchile.sh "${HOME}/uchile.bkp.$(date +"%Y.%m.%d_%H.%M.%S").sh"
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
#_uchile_get_repository "misc/graveyard/old_page"          "https://github.com/uchile-robotics-graveyard/bender_page" # deprecated
#_uchile_get_repository "misc/graveyard/BenderCode"        "https://github.com/uchile-robotics/BenderCode"            # big repo. just slows down installation
#_uchile_get_repository "misc/graveyard/BenderCode_Indigo" "https://github.com/uchile-robotics/BenderCode_Indigo"     # big repo. just slows down installation

# page
#_uchile_get_repository "misc/webpage" "https://github.com/uchile-robotics/uchile-robotics.github.io"

# maqui_system
_uchile_get_repository "misc/maqui_system" "https://github.com/uchile-robotics/maqui_system"


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

# _uchile_get_repository "pkgs/soft_ws/uchile_hri"          "https://github.com/uchile-robotics/uchile_hri"
# _uchile_get_repository "pkgs/soft_ws/uchile_navigation"   "https://github.com/uchile-robotics/uchile_navigation"
# _uchile_get_repository "pkgs/soft_ws/uchile_manipulation" "https://github.com/uchile-robotics/uchile_manipulation"
_uchile_get_repository "pkgs/soft_ws/uchile_perception"   "https://github.com/uchile-robotics/uchile_perception"

_uchile_get_repository "pkgs/high_ws/uchile_high"         "https://github.com/uchile-robotics/uchile_high"
# _uchile_get_repository "pkgs/high_ws/maqui_bringup"       "https://github.com/uchile-robotics/maqui_bringup"
# _uchile_get_repository "pkgs/high_ws/bender_bringup"      "https://github.com/uchile-robotics/bender_bringup"


## forks
## ----------------------------------------------------------------------------

# transitory stuff
# This package is due the fact that is not relased on ros melodic yet
_uchile_get_repository "pkgs/forks_ws/humanoid_msgs"      "https://github.com/ahornung/humanoid_msgs"					   "develop"


# pepper stuff
_uchile_get_repository "pkgs/forks_ws/pepper/nao_robot"            "https://github.com/uchile-robotics-forks/nao_robot"            "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/naoqi_bridge"         "https://github.com/uchile-robotics-forks/naoqi_bridge"         "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/naoqi_bridge_msgs"    "https://github.com/uchile-robotics-forks/naoqi_bridge_msgs"    "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/naoqi_dcm_driver"     "https://github.com/uchile-robotics-forks/naoqi_dcm_driver"     "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/naoqi_driver"         "https://github.com/uchile-robotics-forks/naoqi_driver"         "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/pepper_dcm_robot"     "https://github.com/uchile-robotics-forks/pepper_dcm_robot"     "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/pepper_meshes"        "https://github.com/uchile-robotics-forks/pepper_meshes"        "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/pepper_moveit_config" "https://github.com/uchile-robotics-forks/pepper_moveit_config" "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/pepper_robot"         "https://github.com/uchile-robotics-forks/pepper_robot"         "develop"
_uchile_get_repository "pkgs/forks_ws/pepper/pepper_virtual"       "https://github.com/uchile-robotics-forks/pepper_virtual"       "feat-melodic"

# # forks: rosaria
_uchile_get_repository "pkgs/forks_ws/rosaria" "https://github.com/uchile-robotics-forks/rosaria" "master"

# # fork: dynamixel_motor
_uchile_get_repository "pkgs/forks_ws/dynamixel_motor" "https://github.com/uchile-robotics-forks/dynamixel_motor" "develop"

# # fork: usb_cam
_uchile_get_repository "pkgs/forks_ws/usb_cam" "https://github.com/uchile-robotics-forks/usb_cam" "develop"

# # fork: urg_node
_uchile_get_repository "pkgs/forks_ws/urg_node" "https://github.com/uchile-robotics-forks/urg_node" "uchile-devel"

# # fork: navigation
_uchile_get_repository "pkgs/forks_ws/navigation" "https://github.com/uchile-robotics-forks/navigation" "melodic-devel"

# #fork: hark sound localization
# # TODO: hark_sound_source_localization: Cannot locate rosdep definition for [hark-ros-hydro]
# _uchile_get_repository "pkgs/forks_ws/hark_sound_localization" "https://github.com/uchile-robotics-forks/hark_sound_localization.git" "master"

# # fork: open_ptrack
# #_uchile_get_repository "pkgs/forks_ws/open_ptrack" "https://github.com/uchile-robotics-forks/open_ptrack" "master"

# #fork: moveit python
_uchile_get_repository "pkgs/forks_ws/moveit_python" "https://github.com/uchile-robotics-forks/moveit_python.git" "master"

# # fork: ChatterBot
_uchile_get_repository "pkgs/forks_ws/ChatterBot" "https://github.com/uchile-robotics-forks/ChatterBot" "master"
_uchile_get_repository "pkgs/forks_ws/chatter-corpus" "https://github.com/uchile-robotics-forks/chatterbot-corpus" "master"
# # fork : YoloV3 Detector
_uchile_get_repository "pkgs/forks_ws/yolov3-detector" "https://github.com/uchile-robotics-forks/yolov3-detector" "master"


# #fork: roboticsgroup gazebo plugin for pepper gazeo
_uchile_get_repository "pkgs/forks_ws/roboticsgroup_gazebo_plugins" "https://github.com/uchile-robotics-forks/roboticsgroup_gazebo_plugins"

# # unset credential helper if needed
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
_uchile_enable_githook "system" "${_hook_template}"
_uchile_enable_githook "pkgs/base_ws/uchile_common"       "${_hook_template}"
_uchile_enable_githook "pkgs/base_ws/uchile_knowledge"    "${_hook_template}"
_uchile_enable_githook "pkgs/base_ws/uchile_tools"        "${_hook_template}"
_uchile_enable_githook "pkgs/base_ws/bender_core"         "${_hook_template}"
_uchile_enable_githook "pkgs/base_ws/maqui_core"          "${_hook_template}"
_uchile_enable_githook "pkgs/soft_ws/uchile_hri"          "${_hook_template}"
_uchile_enable_githook "pkgs/soft_ws/uchile_navigation"   "${_hook_template}"
_uchile_enable_githook "pkgs/soft_ws/uchile_manipulation" "${_hook_template}"
_uchile_enable_githook "pkgs/soft_ws/uchile_perception"   "${_hook_template}"
_uchile_enable_githook "pkgs/high_ws/uchile_high"         "${_hook_template}"
_uchile_enable_githook "pkgs/high_ws/maqui_bringup"       "${_hook_template}"
_uchile_enable_githook "pkgs/high_ws/bender_bringup"      "${_hook_template}"

unset _hook_template


## ======================================================
## LINK WORKSPACES
## ======================================================

printf "\n\n ============ Linking Workspaces ============ \n"

bash "${TMP_SYSTEM_DIR}"/install/link_repositories.bash "${framework_path}" "bender"
bash "${TMP_SYSTEM_DIR}"/install/link_repositories.bash "${framework_path}" "maqui"
bash "${TMP_SYSTEM_DIR}"/install/link_repositories.bash "${framework_path}" "all"


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
