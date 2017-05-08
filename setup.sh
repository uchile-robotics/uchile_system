#!/bin/sh

# TODO: enable the framework for ssh connections
# TODO: poder configurar system de forma simple. con lo elemental solamente.
# TODO: system multirobot
# TODO: check user vars are defined or use defaults


## SYSTEM PRE CONFIGURATION CHECKS
## ##########################################

# prevent multiple executions
_currshell=$(ps -p$$ -ocmd=)
if [ "$ROBOT_FRAMEWORK_TWICE_LOAD_CHECK" = true ]; then
    if [ "$_currshell" = "$ROBOT_FRAMEWORK_LOADED_SHELL" ]; then
        echo "You are loading this script twice. Please update your"
        echo ".bashrc/.zshrc files and fix this problem. See the "
        echo "system/README.md file to reconfigure your shell environment."
        unset _currshell
        return 0
    fi
fi
export ROBOT_FRAMEWORK_TWICE_LOAD_CHECK=true
export ROBOT_FRAMEWORK_LOADED_SHELL="$_currshell"

# OBS: Some configurations are meant for outdated machines
# NOTE FOR FUTURE ADMINS: Update this configuration loading for
# release 2.0.
if [ -z "$CATKIN_SHELL" ]; then
    # WARNING: THIS BEHAVIOR IS VALID FOR SSH CONNECTIONS!.
    # OBS: THIS WARNING IS BENDER SPECIFIC!. This problem should only arise on
    # pre MAQUI installations
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
    unset _currshell
    return 0
fi
if [ ! "$CATKIN_SHELL" = "bash" ] && [ ! "$CATKIN_SHELL" = "zsh" ]; then
    echo "Sorry, but the uch robot framework is only designed for bash and zsh shells."
    echo "The framework will not be loaded. Bye"
    unset _currshell
    return 0
fi

# prevent running with an incorrect shell environment
if [ "$_currshell" = "bash" ] && [ ! "$CATKIN_SHELL" = "bash" ]; then
    echo "Attempt to load the uch robot framework for zsh shells on bash."
    echo "Please, source the correct file: setup.bash"
    echo "See also: system/README.md"
    unset _currshell
    return 0
fi
if [ "$_currshell" = "/usr/bin/zsh" ] && [ ! "$CATKIN_SHELL" = "zsh" ]; then
    echo "Attempt to load the uch robot framework for bash shells on zsh."
    echo "Please, source the correct file: setup.zsh"
    echo "See also: system/README.md"
    unset _currshell
    return 0
fi
unset _currshell


## COMPATIBILITY LAYER WITH SYSTEM v1.0
## ########################################################
# As the title describes, this section is meant for old
# system installations and creates a compatibility layer
# between v1.0 on bitbucket (bender_system) and v2.0 on 
# github (uch_system)
#
# mayor compatibility issues are:
# - rename env vars from BENDER_ to ROBOT_
# - workspace layout changes:
#   - ./bender_system/ --> ./system/
#   - ./install/       --> ./deps/
#   - ./forks_ws/ --> ./ros/forks_ws/
#   - ./base_ws/  --> ./ros/base_ws/
#   - ./soft_ws/  --> ./ros/soft_ws/
#   - ./high_ws/  --> ./ros/high_ws/
#   - ./bender_code_graveyard/ --> ./misc/code_graveyard/
#   - ./bender_embedded/       --> ./misc/embedded/
#   - ./wiki/                  --> ./misc/wiki/
#
ROBOT_COMPAT_MODE=false
ROBOT_COMPAT_PREFIX=ROBOT
if [ -z "$ROBOT_WS" ]; then
    ROBOT_COMPAT_MODE=true
    ROBOT_COMPAT_PREFIX=BENDER
    printf "You are running in system compatibility mode with bender_system v1.0.\n."
    printf "Please update your robot installation. See the system README.md file.\n."

    export ROBOT_SHELL_CFG="$BENDER_SHELL_CFG"
    export ROBOT_WS="$BENDER_WS"

    # this var is user defined on v2.0
    export ROBOT="bender"
fi



## SYSTEM USER CONFIGURATIONS
## ########################################################

# load configs
if [ -z "$ROBOT_SHELL_CFG" ]; then
    # this mode is intended for v0.0 users
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
    # mode for v1.0 and newer
    if [ -e "$ROBOT_SHELL_CFG" ]; then
        . "$ROBOT_SHELL_CFG"
    else
        echo "Sorry, the ${ROBOT_COMPAT_PREFIX}_SHELL_CFG is set, but the configuration file cannot be read."
        echo "Configuration file: $ROBOT_SHELL_CFG"
        echo "Please, set this to the path of the <robot>.sh script."
        echo "e.g: \"$HOME/bender.sh\""
        echo "UCH Robot workspace will not be configured."
        return 0
    fi
fi

## checks
## -----------------------------------------

# ROBOT
if [ -z "$ROBOT" ]; then
    echo "Sorry, the ${ROBOT_COMPAT_PREFIX} env variable is not set."
    return 0
fi

# ROBOT_WS
if [ -z "$ROBOT_WS" ]; then
    echo "Sorry, the ${ROBOT_COMPAT_PREFIX}_WS env variable is not set."
    return 0
fi
if [ ! -d "$ROBOT_WS" ]; then
    echo "Sorry, the ${ROBOT_COMPAT_PREFIX}_WS env variable is set. But is not a valid directory."
    echo "Found ${ROBOT_COMPAT_PREFIX}_WS=\'$ROBOT_WS\'"
    return 0
fi

## SYSTEM DEFS
## ##########################################

# paths
export ROBOT_SYSTEM="$ROBOT_WS"/system
if ROBOT_COMPAT_MODE; then
    export ROBOT_SYSTEM="$ROBOT_WS"/bender_system
fi

# contact
export ROBOT_SYSTEM_ADMIN="mpavezb (matias pavez)"
export ROBOT_EMAIL_CONTACT="bender.contacto@gmail.com"
export ROBOT_EMAIL_DEVELOP="bender.devel@gmail.com"


# git hooks
export GITHOOKS_PATH="$ROBOT_SYSTEM/hooks/hooks"

# ros console output format
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'


# (compatibility with repos which depend on v1.0)
# avoid missing variables on outdated repos
if ! ROBOT_COMPAT_MODE; then
    export BENDER_WS="$ROBOT_WS"
    export BENDER_SYSTEM="$ROBOT_SYSTEM"
fi

## ROS
## ##########################################

# manage network configurations
# ----------------------------------------
if [ "$ROBOT_NET_BY_SSH" = "YES" ]; then
    . "$ROBOT_SYSTEM"/env/network-defs.sh
fi

if [ "$ROBOT_NET_ENABLE" = true ]; then
    
    . "$ROBOT_SYSTEM"/env/network-defs.sh

elif [ "$ROBOT_NET_WARN" = true ]; then

    _caller_script=
    if command -v caller >/dev/null 2>&1 ; then
        _caller_script=" on the framework setup file ($(caller | awk '{print $2}'))"
    fi
    
    printf "\e[33m[WARNING]: The Bender Framework has been launched in offline mode.
    Maybe you should set '%s_NET_ENABLE=true'%s
    and run another shell.

    If you want to stop this and all network warnings, 
    set: '%s_NET_WARN=false'%s.\e[0m\n\n" "$ROBOT_COMPAT_PREFIX" "$_caller_script" "$ROBOT_COMPAT_PREFIX" "$_caller_script"

    _sound_file="$ROBOT_SYSTEM"/assets/network_false.wav
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
#. "$ROBOT_WS"/forks_ws/devel/setup.sh

# base_ws
#. "$ROBOT_WS"/base_ws/devel/setup.sh

# soft_ws
#. "$ROBOT_WS"/soft_ws/devel/setup.sh

# high_ws
. "$ROBOT_WS"/high_ws/devel/setup.sh


## BENDER FRAMEWORK
## ##########################################

# bash functionalities
. "$ROBOT_SYSTEM"/shell/setup.sh
