#!/bin/bash
#
# ## installation instructions
# #
# # just run this script anywhere (anywhere but the same workspace path!)
# #
# $ chmod +x bender_framework_install.bash
# $ ./bender_framework_install
#
#
# TODO:
# - agregar manejo de fallo de update de submodulos
# - catkin_make al final


printf "###########################################\n"
printf " BENDER WORKSPACE INSTALLER \n"
printf "###########################################\n"

## ======================================================
## UTILITIES
## ======================================================

cd $(dirname "$BASH_SOURCE")
source "helpers.bash"


## ======================================================
## PRE CHECKS
## ======================================================
printf "\n"
printf " ============ Pre checks: ============ \n"

# ROS is installed
if ! _bender_installer_check_rosindigo; then
    printf "Please, install ROS Indigo (ros-indigo-ros-base) before proceeding.\n"
    exit 1
fi


## ======================================================
## USERDATA
## ======================================================
printf "\n"
printf "\n"
printf " ============ USERDATA: ============ \n"
printf "\n"

# select a folder were to install the framework (default: /home/user/bender_ws)
printf " ------- Framework Location: -------\n"
_bender_installer_ask_framework_path "$HOME/bender_ws"
printf " - Using path: %s\n" "$framework_path"
printf "\n"

# ask credentials
printf " ------- Repository Credentials: -------\n"
username=""
password=""
_bender_installer_ask_single_username
use_credentials=true
if [ -z "$username" ]; then
    use_credentials=false
fi

## PREPARE WORKSPACE
## ========================================
# at this point:
# - framework_path contains the fullpath to a (existent) folder
# - framework_path could be a non empty directory
# - ROS indigo is installed under /opt/ros/indigo
#
printf "\n"
printf "\n"
printf " ============ Workspace Preparation: ============ \n"
printf "\n"


## create workspaces and create the ROS overlay
## ---------------------------------------------

# ROS baseline 
source /opt/ros/indigo/setup.bash

# forks_ws overlays ROS baseline
_bender_installer_reset_ws "$framework_path"/forks_ws

# base_ws overlays forks_ws
_bender_installer_reset_ws "$framework_path"/base_ws

# soft_ws overlays base_ws
_bender_installer_reset_ws "$framework_path"/soft_ws

# high_ws overlays soft_ws
_bender_installer_reset_ws "$framework_path"/high_ws



## GET REPOSITORIES
## ========================================
#
# repos are not cloned again if they already exists
#
printf "\n"
printf "\n"
printf " ============ Retrieving Repositories ============ \n"

# WIKI
_repo_path="$framework_path"/wiki
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_system.git/wiki
_bender_installer_get_repository "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

	
# bender_system
_repo_path="$framework_path"/bender_system
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_system.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

# base layer
_repo_path="$framework_path"/base_ws/src
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_base_layer
_bender_installer_get_repository_for_ws "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

# soft layer
_repo_path="$framework_path"/soft_ws/src
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_soft_layer
_bender_installer_get_repository_for_ws "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

# high layer
_repo_path="$framework_path"/high_ws/src
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_high_layer
_bender_installer_get_repository_for_ws "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

# bender_code_graveyard
_repo_path="$framework_path"/bender_code_graveyard
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_code_graveyard.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

# bender_embedded
_repo_path="$framework_path"/bender_embedded
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_embedded.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

# forks: rosaria
_repo_path="$framework_path"/forks_ws/src/rosaria
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_fork_rosaria.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "$use_credentials" "$username" "$password"

# fork: dynamixel_motor
cd "$framework_path"/forks_ws/src
git clone https://github.com/uchile-robotics/dynamixel_motor.git
cd "$framework_path"/forks_ws/src/dynamixel_motor
git checkout develop

# fork: usb_cam
cd "$framework_path"/forks_ws/src
git clone https://github.com/uchile-robotics/usb_cam.git
cd "$framework_path"/forks_ws/src/usb_cam
git checkout 0.3.4

# fork: urg_node
cd "$framework_path"/forks_ws/src
git clone https://github.com/uchile-robotics/urg_node.git
cd "$framework_path"/forks_ws/src/urg_node
git checkout 0.1.9

unset use_credentials username password
unset _repo_url _repo_path


## ENABLE HOOKS
## ==========================================
printf "\n"
printf "\n"
printf " ============ Enabling GIT HOOKS ============ \n"

_hook_file="$framework_path"/bender_system/hooks/hooks/pre-commit


## bender_system
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/bender_system/.git/hooks


## bender_base_layer
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/base_ws/src/.git/hooks

# submodule: bender_knowledge
_bender_installer_enable_hook "$framework_path"/base_ws/src/.git/modules/bender_knowledge/hooks


## bender_soft_layer
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/hooks

# submodule: bender_hri
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_hri/hooks

# submodule: bender_tools
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_tools/hooks

# submodule: bender_manipulation
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_manipulation/hooks

# submodule: bender_navigation
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_navigation/hooks

# submodule: bender_perception
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_perception/hooks


## bender_high_layer
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/high_ws/src/.git/hooks



## bender_code_graveyard
## ---------------------------------------------------
# no queremos hooks en el graveyard!, es codigo que no vale la pena arreglar
#_bender_installer_enable_hook "$framework_path"/bender_code_graveyard/.git/hooks

## bender_embedded
## ---------------------------------------------------
_bender_installer_enable_hook "$framework_path"/bender_embedded/.git/hooks

## forks: rosaria
## ---------------------------------------------------
_bender_installer_enable_hook "$framework_path"/forks_ws/src/rosaria/.git/hooks

unset _hook_file



## BENDER.SH
## ==========================================
printf "\n"
printf "\n"
printf " ============ Setting up bender.sh ============ \n"

# bender.sh
# -----------------------------
printf " - setting up file: $HOME/bender.sh\n"
template="$framework_path"/bender_system/templates/bender.sh
cp -f "$template" "$HOME"/bender.sh
unset template


# END
# ----------------------------
printf "\n"
printf "The installation is almost ready!. Just follow the installation\n"
printf "instructions on the bender_system README.md file.\n"
printf "\n"

unset framework_path

printf "###########################################\n"
printf " DONE! \n"
printf "###########################################\n"
