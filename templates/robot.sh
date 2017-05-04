#!/bin/sh
#
# Shell settings for the <ROBOT_LOWER> framework.
#
# Do not source this file into your *rc files.
# This file is automagically sourced whenever a new shell is opened.
#
#
# To enable the framework, you must setup the following on your rc file:
# 
# # workspace location
# export <ROBOT_UPPER>_WS=<FRAMEWORK_PATH>
#
# # workspace configuration file
# export <ROBOT_UPPER>_SHELL_CFG="$HOME"/<ROBOT_LOWER>.sh
#
# # source the setup file
# . "$<ROBOT_UPPER>_WS"/system/setup.bash  # (on .bashrc : bash only)
# . "$<ROBOT_UPPER>_WS"/system/setup.zsh   # (on .zshrc  : zsh only )
#
#

## <ROBOT_UPPER> FRAMEWORK SETTINGS
## ==========================================

# networking settings
# -------------------------------------------
# available names are:
# - chest / nav / vision

# Your IP address or name
export <ROBOT_UPPER>_NET_IP="SET-THIS-VALUE"

# ROS MASTER IP address or name
export <ROBOT_UPPER>_NET_MASTER="chest"

# Enable ROS networking (true/false)
export <ROBOT_UPPER>_NET_ENABLE=false

# (dis)plays a warning when working in offline mode (true/false)
export <ROBOT_UPPER>_NET_WARN=false
