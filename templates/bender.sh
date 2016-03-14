#!/bin/sh


## BENDER FRAMEWORK SETTINGS
## ==========================================<

# bender workspace location
export BENDER_WS="$HOME"/bender_ws


# networking settings
# -------------------------------------------

# Your IP address
export BENDER_NET_IP="192.168.1.1"

# ROS MASTER IP address
export BENDER_NET_MASTER="192.168.1.10"

# Enable ROS networking (true/false)
export BENDER_NET_ENABLE=false

# (dis)plays a warning whe working in offline mode (true/false)
export BENDER_NET_WARN=false



## LOAD FRAMEWORK
## ==========================================<

# bender framework
. "$BENDER_WS"/bender_system/setup.sh
