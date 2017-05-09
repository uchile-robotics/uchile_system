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
# export UCH_WS=<FRAMEWORK_PATH>
#
# # workspace configuration file
# export UCH_SHELL_CFG="$HOME"/<ROBOT_LOWER>.sh
#
# # source the setup file
# . "$UCH_WS"/system/setup.bash  # (on .bashrc : bash only)
# . "$UCH_WS"/system/setup.zsh   # (on .zshrc  : zsh only )
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
export UCH_NET_IP="SET-THIS-VALUE"

# ROS MASTER IP address or name
export UCH_NET_MASTER="chest"

# Enable ROS networking (true/false)
export UCH_NET_ENABLE=false

# (dis)plays a warning when working in offline mode (true/false)
export UCH_NET_WARN=false
