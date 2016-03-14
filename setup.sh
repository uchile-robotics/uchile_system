#!/bin/sh

## SYSTEM DEFS
## ##########################################

# paths
export BENDER_SYSTEM="$BENDER_WS"/bender_system

# contact
export BENDER_SYSTEM_ADMIN="<mpavez> - matias pavez"
export BENDER_EMAIL_CONTACT="bender.contacto@gmail.com"
export BENDER_EMAIL_DEVELOP="bender.devel@gmail.com"


# git hooks
export GITHOOKS_PATH="$BENDER_SYSTEM/hooks/hooks"

# ros console output format
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'


## ROS
## ##########################################

# manage network configurations
# ----------------------------------------
if [ "$BENDER_NET_BY_SSH" = "YES" ]; then
    . "$BENDER_SYSTEM"/env/network-defs.sh
else
    export CATKIN_SHELL=bash
fi

if [ "$BENDER_NET_ENABLE" = true ]; then
    
    echo "enbled"
    . "$BENDER_SYSTEM"/env/network-defs.sh

elif [ "$BENDER_NET_WARN" = true ]; then

    _caller_script=
    if command -v caller >/dev/null 2>&1 ; then
        _caller_script=" on the framework setup file ($(caller | awk '{print $2}'))"
    fi
    
    printf "\e[33m[WARNING]: The Bender Framework has been launched in offline mode.
    Maybe you should set 'BENDER_NET_ENABLE=true'%s
    and run another shell.

    If you want to stop this and all network warnings, 
    set: 'BENDER_NET_WARN=false'%s.\e[0m\n\n" "$_caller_script" "$_caller_script"

    _sound_file="$BENDER_SYSTEM"/assets/network_false.wav
    if [ -f "$_sound_file" ]; then
        aplay "$_sound_file" >/dev/null 2>&1
    else
        printf "\e[33m[WARNING]: I was supposed to play a sound, but the required .wav file is missing:
        %s\e[0m\n\n" "$_sound_file"
    fi
    unset _sound_file
fi


# manage ros framework
# ---------------------------------------
#
# it's very very important to source the setup.sh files and not the setup.bash ones!
# this way, the configuration is nor messed up when dealing with ssh.

# ROS
#. /opt/ros/indigo/setup.sh

# forks
#. "$BENDER_WS"/forks_ws/devel/setup.sh

# base_ws
#. "$BENDER_WS"/base_ws/devel/setup.sh

# soft_ws
#. "$BENDER_WS"/soft_ws/devel/setup.sh

# high_ws
. "$BENDER_WS"/high_ws/devel/setup.sh


## BENDER FRAMEWORK
## ##########################################

# bash functionalities
. "$BENDER_SYSTEM"/bash/setup.sh
