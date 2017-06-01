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
    echo "the uchile.sh file to source the proper shell script:"
    echo "setup.bash or setup.zsh, for bash or zsh users."
    echo ""
    echo ""
    echo "THE UCHILE ROS WORKSPACE WILL NOT BE LOADED. PLEASE, UPDATE YOUR SETTINGS"
    echo "AS FOLLOWS:"
    echo "$ cp $HOME/uchile.sh $HOME/uchile.sh.bkp"
    echo "$ cp $HOME/uchile_ws/system/templates/uchile.sh $HOME/uchile.sh"
    echo ""
    echo "Then edit your .bashrc:"
    echo "----------------------------------------------------------------"
    echo '# UChile ROS Workspace settings: location, configs and setup script.'
    echo 'export UCHILE_WS="$HOME"/uchile_ws'
    echo 'export UCHILE_SHELL_CFG="$HOME"/uchile.sh'
    echo '. "$UCHILE_WS"/system/setup.bash'
    echo "----------------------------------------------------------------"
    echo ""
    echo "For zsh shells, edit your .zshrc and use the system/setup.zsh file."
    echo "More info on the system/README.md file."
    echo ""
    echo "Bye."
    unset _currshell
    return 0
fi
if [ ! "$CATKIN_SHELL" = "bash" ] && [ ! "$CATKIN_SHELL" = "zsh" ]; then
    echo "Sorry, but the UChile ROS Framework is only designed for bash and zsh shells."
    echo "The framework will not be loaded. Bye"
    unset _currshell
    return 0
fi

# prevent running with an incorrect shell environment
if [ "$_currshell" = "bash" ] && [ ! "$CATKIN_SHELL" = "bash" ]; then
    echo "Attempt to load the UChile ROS Framework for zsh shells on bash."
    echo "Please, source the correct file: setup.bash"
    echo "See also: system/README.md"
    unset _currshell
    return 0
fi
if [ "$_currshell" = "/usr/bin/zsh" ] && [ ! "$CATKIN_SHELL" = "zsh" ]; then
    echo "Attempt to load the UChile ROS Framework for bash shells on zsh."
    echo "Please, source the correct file: setup.zsh"
    echo "See also: system/README.md"
    unset _currshell
    return 0
fi
unset _currshell



## SYSTEM USER CONFIGURATIONS
## ########################################################

# load configs
if [ -z "$UCHILE_SHELL_CFG" ]; then
    echo "Sorry, the UCHILE_SHELL_CFG env variable is not set."
    echo "Please, set this to the path of the uchile.sh script."
    echo "e.g: \"$HOME/uchile.sh\""
    echo "UChile ROS Framework will not be configured."
    return 0
else
    # mode for v1.0 and newer
    if [ -e "$UCHILE_SHELL_CFG" ]; then
        . "$UCHILE_SHELL_CFG"
    else
        echo "Sorry, the UCHILE_SHELL_CFG is set, but the configuration file cannot be read."
        echo "Configuration file: $UCHILE_SHELL_CFG"
        echo "Please, set this to the path of the uchile.sh script."
        echo "e.g: \"$HOME/uchile.sh\""
        echo "UChile ROS Framework will not be configured."
        return 0
    fi
fi


## checks
## -----------------------------------------

# ROBOT
if [ -z "$UCHILE_ROBOT" ]; then
    echo "Sorry, the UCHILE_ROBOT env variable is not set."
    return 0
fi

# UCHILE_WS
if [ -z "$UCHILE_WS" ]; then
    echo "Sorry, the UCHILE_WS env variable is not set."
    return 0
fi
if [ ! -d "$UCHILE_WS" ]; then
    echo "Sorry, the UCHILE_WS env variable is set. But is not a valid directory."
    echo "Found UCHILE_WS=\'$UCHILE_WS\'"
    return 0
fi


## SYSTEM DEFS
## ##########################################

# paths
export UCHILE_SYSTEM="$UCHILE_WS"/system
export UCHILE_ROS_WS="$UCHILE_WS/ros/$UCHILE_ROBOT"
export UCHILE_DEPS_WS="$UCHILE_WS"/deps
export UCHILE_MISC_WS="$UCHILE_WS"/misc
export UCHILE_PKGS_WS="$UCHILE_WS"/pkgs


# contact
export UCHILE_SYSTEM_ADMIN="mpavezb (matias pavez)"
export UCHILE_EMAIL_CONTACT="bender.contacto@gmail.com"
export UCHILE_EMAIL_DEVELOP="bender.devel@gmail.com"

# git hooks
export GITHOOKS_PATH="$UCHILE_SYSTEM/hooks"

# ros console output format
export ROSCONSOLE_FORMAT='[${severity}] [${node}]: ${message}'

# --------------------------------------------------------
# (compatibility with repos which depend on v1.9 env vars)
export BENDER_WS="$UCHILE_ROS_WS"
export BENDER_SYSTEM="$UCHILE_SYSTEM"
# --------------------------------------------------------


## ROS
## ##########################################

# manage network configurations
# ----------------------------------------
if [ "$UCHILE_NET_BY_SSH" = true ]; then
    . "$UCHILE_SYSTEM"/env/network-defs.sh
fi

if [ "$UCHILE_NET_ENABLE" = true ]; then
    . "$UCHILE_SYSTEM"/env/network-defs.sh

elif [ "$UCHILE_NET_WARN" = true ]; then

    _caller_script=
    if command -v caller >/dev/null 2>&1 ; then
        _caller_script=" on the framework setup file ($(caller | awk '{print $2}'))"
    fi
    
    printf "\e[33m[WARNING]: The UChile ROS Framework has been launched in offline mode.
    Maybe you should set 'UCHILE_NET_ENABLE=true'%s
    and run another shell.

    If you want to stop this and all network warnings, 
    set: 'UCHILE_NET_WARN=false'%s.\e[0m\n\n" "$_caller_script" "$_caller_script"

    _sound_file="$UCHILE_SYSTEM"/assets/network_false.wav
    if [ -f "$_sound_file" ]; then
        aplay "$_sound_file" >/dev/null 2>&1
    else
        printf "\e[33m[WARNING]: I was supposed to play a sound, but the required .wav file is missing:
        %s\e[0m\n\n" "$_sound_file"
    fi
    unset _sound_file
    unset _caller_script
fi


# manage ros framework
# ---------------------------------------
#
# it's very very important to source the setup.sh files and not the setup.bash ones!
# this way, the configuration is not messed up when dealing with ssh.

setup_file="$UCHILE_ROS_WS"/high_ws/devel/setup.sh
if [ -e "${setup_file}" ]; then
    . "${setup_file}"
else
    printf "[UChile ROS Framework]: Error: Missing setup.sh file on high_ws.\n"
    printf " - Required file: %s\n" "${setup_file}"
    printf " - My advice is to run the workspace installer script:\n"
    printf "\n"
    printf "    bash $UCHILE_SYSTEM/install/ws_installer.bash\n"
    printf "\n"
    printf "Then reopen the terminal and try again.\n"
    printf "\n"
    printf "ROS Will not be sourced.\n"
    return 1
fi
unset setup_file



## UChile ROS Framework Tools
## ##########################################

# shell functionalities
. "$UCHILE_SYSTEM"/shell/setup.sh
