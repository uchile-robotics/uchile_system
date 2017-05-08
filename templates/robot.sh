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
# export ROBOT_WS=<FRAMEWORK_PATH>
#
# # workspace configuration file
# export ROBOT_SHELL_CFG="$HOME"/<ROBOT_LOWER>.sh
#
# # source the setup file
# . "$ROBOT_WS"/system/setup.bash  # (on .bashrc : bash only)
# . "$ROBOT_WS"/system/setup.zsh   # (on .zshrc  : zsh only )
#
#

## ROBOT FRAMEWORK SETTINGS
## ==========================================

# robot settings
# ------------------------------------------- 

export ROBOT="<ROBOT_LOWER>"


# networking settings
# -------------------------------------------
# available names are:
# - chest / nav / vision

# Your IP address or name
export ROBOT_NET_IP="SET-THIS-VALUE"

# ROS MASTER IP address or name
export ROBOT_NET_MASTER="chest"

# Enable ROS networking (true/false)
export ROBOT_NET_ENABLE=false

# (dis)plays a warning when working in offline mode (true/false)
export ROBOT_NET_WARN=false
