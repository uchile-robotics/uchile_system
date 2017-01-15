#!/bin/sh

# TODO: enable the framework for ssh connections

## SYSTEM CONFIGURATIONS
## ##########################################

# prevent multiple executions
if [ "$BENDER_FRAMEWORK_TWICE_LOAD_CHECK" = true ]; then
    echo "You are loading this script twice. Please update your"
    echo ".bashrc/.zshrc files and fix this problem. See the "
    echo "bender_system/README.md file to reconfigure your shell"
    echo "environment."
    return 0
fi
export BENDER_FRAMEWORK_TWICE_LOAD_CHECK=true

# OBS: Some configurations are meant for outdated machines
# NOTE FOR FUTURE ADMINS: Update this configuration loading for
# release 2.0.
if [ -z "$CATKIN_SHELL" ]; then
    # WARNING: THIS BEHAVIOR IS VALID FOR SSH CONNECTIONS!.
    echo "Sorry, the CATKIN_SHELL env variable is not set. I assume you are "
    echo "loading this script as setup.sh. Please, modify your *rc file or "
    echo "the bender.sh file to source the proper shell script:"
    echo "setup.bash or setup.zsh, for bash or zsh users."
    echo ""
    echo ""
    echo "THE BENDER WORKSPACE WILL NOT BE LOADED. PLEASE, UPDATE YOUR SETTINGS"
    echo "AS FOLLOWS:"
    echo "$ cp $HOME/bender.sh $HOME/bender.sh.bkp"
    echo "$ cp $HOME/bender_ws/bender_system/templates/bender.sh $HOME/bender.sh"
    echo ""
    echo "Then edit your .bashrc:"
    echo "1st: REMOVE the old bender lines (e.g: source bender.sh ....)"
    echo "2nd: Append the following:"
    echo "----------------------------------------------------------------"
    echo '# Bender Workspace settings: location, configs and setup script.'
    echo 'export BENDER_WS="$HOME"/bender_ws'
    echo 'export BENDER_SHELL_CFG="$HOME"/bender.sh'
    echo '. "$BENDER_WS"/bender_system/setup.bash'
    echo "----------------------------------------------------------------"
    echo ""
    echo "For zsh shells, edit your .zshrc and use the bender_system/setup.zsh file."
    echo "More info on the bender_system/README.md file."
    echo ""
    echo "Bye."
    return 0
fi
if [ ! "$CATKIN_SHELL" = "bash" ] && [ ! "$CATKIN_SHELL" = "zsh" ]; then
    echo "Sorry, but the bender framework is only designed for bash and zsh shells."
    echo "The framework will not be loaded. Bye"
    return 0
fi

# prevent running with an incorrect shell environment
_currshell=$(ps -p$$ -ocmd=)
if [ "$_currshell" = "bash" ] && [ ! "$CATKIN_SHELL" = "bash" ]; then
    echo "Attempt to load the bender framework for zsh shells on bash."
    echo "Please, source the correct file: setup.bash"
    echo "See also: bender_system/README.md"
    return 0
fi
if [ "$_currshell" = "/usr/bin/zsh" ] && [ ! "$CATKIN_SHELL" = "zsh" ]; then
    echo "Attempt to load the bender framework for bash shells on zsh."
    echo "Please, source the correct file: setup.zsh"
    echo "See also: bender_system/README.md"
    return 0
fi
unset _currshell


# load configs
if [ -z "$BENDER_SHELL_CFG" ]; then
    DEFAULT_CFG="$HOME"/bender.sh
    if [ -e "$DEFAULT_CFG" ]; then
        . "$DEFAULT_CFG"
    else
        echo "Sorry, the BENDER_SHELL_CFG env variable is not set."
        echo "Please, set this to the path of the bender.sh script."
        echo "e.g: \"$HOME/bender.sh\""
        echo "Bender workspace will not be configured."
        return 0
    fi
else
    if [ -e "$BENDER_SHELL_CFG" ]; then
        . "$BENDER_SHELL_CFG"
    else
        echo "Sorry, the BENDER_SHELL_CFG is set, the configuration file cannot be read."
        echo "Configuration file: $BENDER_SHELL_CFG"
        echo "Please, set this to the path of the bender.sh script."
        echo "e.g: \"$HOME/bender.sh\""
        echo "Bender workspace will not be configured."
        return 0
    fi
fi


## SYSTEM DEFS
## ##########################################

# paths
export BENDER_SYSTEM="$BENDER_WS"/bender_system

# contact
export BENDER_SYSTEM_ADMIN="mpavez (matias pavez)"
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
fi

if [ "$BENDER_NET_ENABLE" = true ]; then
    
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
. "$BENDER_SYSTEM"/shell/setup.sh
