#!/bin/sh

# TODO: enable the framework for ssh connections
# TODO: poder configurar system de forma simple. con lo elemental solamente.
# TODO: system multirobot
# TODO: check user vars are defined or use defaults


## SYSTEM PRE CONFIGURATION CHECKS
## ##########################################

# prevent multiple executions
_currshell=$(ps -p$$ -ocmd=)
if [ "$UCHILE_FRAMEWORK_TWICE_LOAD_CHECK" = true ]; then
    if [ "$_currshell" = "$UCHILE_FRAMEWORK_LOADED_SHELL" ]; then
        echo "You are loading this script twice. Please update your"
        echo ".bashrc/.zshrc files and fix this problem. See the "
        echo "system/README.md file to reconfigure your shell environment."
        unset _currshell
        return 0
    fi
fi
export UCHILE_FRAMEWORK_TWICE_LOAD_CHECK=true
export UCHILE_FRAMEWORK_LOADED_SHELL="$_currshell"

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
    echo "Sorry, but the uchile robot framework is only designed for bash and zsh shells."
    echo "The framework will not be loaded. Bye"
    unset _currshell
    return 0
fi

# prevent running with an incorrect shell environment
if [ "$_currshell" = "bash" ] && [ ! "$CATKIN_SHELL" = "bash" ]; then
    echo "Attempt to load the uchile robot framework for zsh shells on bash."
    echo "Please, source the correct file: setup.bash"
    echo "See also: system/README.md"
    unset _currshell
    return 0
fi
if [ "$_currshell" = "/usr/bin/zsh" ] && [ ! "$CATKIN_SHELL" = "zsh" ]; then
    echo "Attempt to load the uchile robot framework for bash shells on zsh."
    echo "Please, source the correct file: setup.zsh"
    echo "See also: system/README.md"
    unset _currshell
    return 0
fi
unset _currshell


## COMPATIBILITY LAYER WITH SYSTEM v1.9
## ########################################################
# As the title describes, this section is meant for old
# system installations and creates a compatibility layer
# between v1.9 on bitbucket (bender_system) and v2.0 on 
# github (uchile_system)
#
# mayor compatibility issues are:
# - rename env vars from BENDER_ to UCHILE_
# - workspace layout changes:
#   - ./bender_system/ --> ./system/
#   - ./install/       --> ./deps/
#   - ./forks_ws/ --> ./ros/forks_ws/
#   - ./base_ws/  --> ./ros/base_ws/
#   - ./soft_ws/  --> ./ros/soft_ws/
#   - ./high_ws/  --> ./ros/high_ws/
#   - ./bender_code_graveyard/ --> ./misc/graveyard/
#   - ./bender_embedded/       --> ./misc/embedded/
#   - ./wiki/                  --> ./misc/wiki/
#
UCHILE_COMPAT_MODE=false
UCHILE_COMPAT_PREFIX=ROBOT
if [ -z "$UCHILE_WS" ]; then
    UCHILE_COMPAT_MODE=true
    UCHILE_COMPAT_PREFIX=BENDER
    printf "You are running in system compatibility mode with bender_system v1.9.\n."
    printf "Please update your robot installation. See the system README.md file.\n."

    export UCHILE_SHELL_CFG="$BENDER_SHELL_CFG"
    export UCHILE_WS="$BENDER_WS"

    # this var is user defined on v2.0
    export ROBOT="bender"
fi



## SYSTEM USER CONFIGURATIONS
## ########################################################

# load configs
if [ -z "$UCHILE_SHELL_CFG" ]; then
    # this mode is intended for v1.0 users
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
    if [ -e "$UCHILE_SHELL_CFG" ]; then
        . "$UCHILE_SHELL_CFG"
    else
        echo "Sorry, the ${UCHILE_COMPAT_PREFIX}_SHELL_CFG is set, but the configuration file cannot be read."
        echo "Configuration file: $UCHILE_SHELL_CFG"
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
    echo "Sorry, the ${UCHILE_COMPAT_PREFIX} env variable is not set."
    return 0
fi

# UCHILE_WS
if [ -z "$UCHILE_WS" ]; then
    echo "Sorry, the ${UCHILE_COMPAT_PREFIX}_WS env variable is not set."
    return 0
fi
if [ ! -d "$UCHILE_WS" ]; then
    echo "Sorry, the ${UCHILE_COMPAT_PREFIX}_WS env variable is set. But is not a valid directory."
    echo "Found ${UCHILE_COMPAT_PREFIX}_WS=\'$UCHILE_WS\'"
    return 0
fi

## SYSTEM DEFS
## ##########################################

# paths
export UCHILE_SYSTEM="$UCHILE_WS"/system
export UCHILE_ROS_WS="$UCHILE_WS"/ros
export UCHILE_DEP_WS="$UCHILE_WS"/deps
export UCHILE_GRAVEYARD="$UCHILE_WS"/misc/graveyard
export UCHILE_EMBEDDED="$UCHILE_WS"/misc/embedded
if UCHILE_COMPAT_MODE; then
    export UCHILE_SYSTEM="$UCHILE_WS"/bender_system
    export UCHILE_ROS_WS="$UCHILE_WS"
    export UCHILE_DEP_WS="$UCHILE_WS"/install
    export UCHILE_GRAVEYARD="$UCHILE_WS"/bender_code_graveyard
    export UCHILE_EMBEDDED="$UCHILE_WS"/bender_embedded
fi

# contact
export UCHILE_SYSTEM_ADMIN="mpavezb (matias pavez)"
export UCHILE_EMAIL_CONTACT="bender.contacto@gmail.com"
export UCHILE_EMAIL_DEVELOP="bender.devel@gmail.com"

# git hooks
export GITHOOKS_PATH="$UCHILE_SYSTEM/hooks/hooks"

# ros console output format
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'


# (compatibility with repos which depend on v1.9)
# avoid missing variables on outdated repos
if ! UCHILE_COMPAT_MODE; then
    export BENDER_WS="$UCHILE_WS"
    export BENDER_SYSTEM="$UCHILE_SYSTEM"
fi

## ROS
## ##########################################

# manage network configurations
# ----------------------------------------
if [ "$UCHILE_NET_BY_SSH" = "YES" ]; then
    . "$UCHILE_SYSTEM"/env/network-defs.sh
fi

if [ "$UCHILE_NET_ENABLE" = true ]; then
    
    . "$UCHILE_SYSTEM"/env/network-defs.sh

elif [ "$UCHILE_NET_WARN" = true ]; then

    _caller_script=
    if command -v caller >/dev/null 2>&1 ; then
        _caller_script=" on the framework setup file ($(caller | awk '{print $2}'))"
    fi
    
    printf "\e[33m[WARNING]: The Bender Framework has been launched in offline mode.
    Maybe you should set '%s_NET_ENABLE=true'%s
    and run another shell.

    If you want to stop this and all network warnings, 
    set: '%s_NET_WARN=false'%s.\e[0m\n\n" "$UCHILE_COMPAT_PREFIX" "$_caller_script" "$UCHILE_COMPAT_PREFIX" "$_caller_script"

    _sound_file="$UCHILE_SYSTEM"/assets/network_false.wav
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
#. "$UCHILE_ROS_WS"/forks_ws/devel/setup.sh

# base_ws
#. "$UCHILE_ROS_WS"/base_ws/devel/setup.sh

# soft_ws
#. "$UCHILE_ROS_WS"/soft_ws/devel/setup.sh

# high_ws
. "$UCHILE_ROS_WS"/high_ws/devel/setup.sh


## UCH ROBOT FRAMEWORK
## ##########################################

# bash functionalities
. "$UCHILE_SYSTEM"/shell/setup.sh
