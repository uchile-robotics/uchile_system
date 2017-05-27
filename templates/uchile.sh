#!/bin/sh
#
# Shell settings for the uchile ROS framework.
#
# Do not source this file into your *rc files.
# This file is automagically sourced whenever a new shell is opened.
#
#
# To enable the UChile ROS Framework, you must setup the following on your rc file:
# 
# # UChile workspace location
# export UCHILE_WS=<FRAMEWORK_PATH>
#
# # workspace configuration file
# export UCHILE_SHELL_CFG="$HOME"/uchile.sh
#
# # source the setup file
# . "$UCHILE_WS"/system/setup.bash  # (on .bashrc : bash only)
# . "$UCHILE_WS"/system/setup.zsh   # (on .zshrc  : zsh only )
#
#

## UCHILE ROS FRAMEWORK SETTINGS
## ==========================================

# robot settings
# ------------------------------------------- 

# available: bender, maqui, all
export UCHILE_ROBOT="bender"


# networking settings
# -------------------------------------------
# available IP names are:
# - chest / nav / vision

# Your IP address or name
export UCHILE_NET_IP="SET-THIS-VALUE"

# ROS MASTER IP address or name
export UCHILE_NET_MASTER="chest"

# Enable ROS networking (true/false)
export UCHILE_NET_ENABLE=false

# (dis)plays a warning when working in offline mode (true/false)
export UCHILE_NET_WARN=false
