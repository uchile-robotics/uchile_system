#!/bin/sh


## BENDER FRAMEWORK SETTINGS
## ==========================================

# bender workspace location
export BENDER_WS="$HOME"/bender_ws


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



## LOAD FRAMEWORK
## ==========================================

# bender framework
. "$BENDER_WS"/bender_system/setup.sh
