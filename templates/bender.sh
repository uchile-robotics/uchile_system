#!/bin/sh
#
# Shell settings for the bender framework.
#
# Do not source this file into your *rc files.
# This file is automagically sourced whenever a new shell is opened.
#
#
# To enable the framework, you must setup the following on your rc file:
# 
# # workspace location
# export BENDER_WS="$HOME"/bender_ws
#
# # workspace configuration file
# export BENDER_SHELL_CFG="$HOME"/bender.sh
#
# # source the setup file
# . "$BENDER_WS"/bender_system/setup.bash  # (on .bashrc : bash only)
# . "$BENDER_WS"/bender_system/setup.zsh   # (on .zshrc  : zsh only )
#
#

## BENDER FRAMEWORK SETTINGS
## ==========================================

# networking settings
# -------------------------------------------
# available names are:
# - chest / nav / vision

# Your IP address or name
export BENDER_NET_IP="SET-THIS-VALUE"

# ROS MASTER IP address or name
export BENDER_NET_MASTER="chest"

# Enable ROS networking (true/false)
export BENDER_NET_ENABLE=false

# (dis)plays a warning when working in offline mode (true/false)
export BENDER_NET_WARN=false
