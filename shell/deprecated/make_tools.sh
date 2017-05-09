#!/bin/sh

bender_clean_ws ()
{
    local ws path user_path

    if [ -z "$1" ] || [ "$1" = "-h" ] || [ "$1" = "--help"  ]; then

        # display help
        cat <<EOF
Synopsis:                
    bender_clean_ws <workspace>

Description:
    It attempts to clean all files which are (catkin_make)generated while 
    building a workspace. This is useful when testing for dependencies
    declaration completeness.

    Specifically, it deletes the build/ and devel/{bin,include,lib,share}
    folders.

Options:
    Use the 'workspace' option to select workspace that will be cleaned.
    
    Supported values are:
        - base   : cleans the base_ws
        - soft   : cleans the soft_ws
        - high   : cleans the high_ws
        - forks  : cleans the forks_ws

    There is no default value!
EOF
        _uch_admin_goodbye
        return 0

    else
        ws="$1"
        user_path="$(pwd)"
        path="$UCH_WS"/"$ws"_ws

        if [ ! -e "$path" ]; then
            echo "Invalid workspace: $1 at $UCH_WS"
            return 1
        fi
        if [ ! -e "$path"/"src"/CMakeLists.txt ]; then
            echo "Invalid workspace: $1 at $UCH_WS"
            echo "Missing CMakeLists.txt at $path/src/"
            return 1
        fi
        if [ ! -e "$path"/"devel"/setup.sh ]; then
            echo "Invalid workspace: $1 at $UCH_WS"
            echo "Missing setup.sh at $path/devel/"
            return 1
        fi

        echo "This workspace will be (make)cleaned: $path"
        if ! _uch_check_user_confirmation ; then
            return 0
        fi

        cd "$path"
        echo " - deleting 'build' ... "         && rm -rf build
        echo " - deleting 'devel/bin' ... "     && rm -rf devel/bin
        echo " - deleting 'devel/include' ... " && rm -rf devel/include
        echo " - deleting 'devel/lib' ... "     && rm -rf devel/lib
        echo " - deleting 'devel/share' ... "   && rm -rf devel/share
        cd "$user_path"
        return 0
    fi
}

# AUN NO ESTAN DISPONIBLES!
return 0

#############################################################################################
#  CATKIN MAKE HELPER FUNCTIONS
#############################################################################################

_uch_make_common()
{

    cd "$BENDER_WORKSPACE"/..
    _make_pkg="$2"

    if [ -z "$2" ]; then
        # build all packages by default
        _make_pkg=$UCH_PACKAGES

    elif [ "$2" = "-h" ] || [ "$2" = "--help"  ]; then

        # display help
        cat <<EOF
        Usage: bender_make [ my-bender-pkg | clean | --eclipse | -h | --help ]

            examples:
            $ bender_make*           : builds all bender packages located on the workspace (and its dependencies)
            $ bender_make* my_pkg    : builds my_pkg and its dependencies
            $ bender_make* clean     : cleans bender workspace
            $ bender_make* -h        :
            $ bender_make* --eclipse : builds code and generates .cproject and .project files for eclipse. Then simply import the <catkin_ws>/build/ folder as 'Existing Code as a Makefile Project' into eclipse.

            if you want a specific build type for cmake, please use one of the followings commands:
            - bender_make_Debug
            - bender_make_RelWithDebInfo
            - bender_make_Release
EOF
        _uch_admin_goodbye
        cd "$OLDPWD"
        return

    elif [ "$2" = "clean" ]; then

        # clean worspace
        echo "cleaning build space... "
        rm -rf build/

        echo "cleaning devel space... "
        rm -rf devel/include
        rm -rf devel/lib
        rm -rf devel/share

        echo "cleaning install space... "
        rm -rf install/

        cd "$OLDPWD"
        return

    elif [ "$2" = "--eclipse" ]; then

        # build all packages and generate eclipse project files
        _make_pkg=$UCH_PACKAGES

        catkin_make -DCMAKE_BUILD_TYPE="$1" --only-pkg-with-deps $_make_pkg --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
        cd "$OLDPWD"
        return
    fi

    catkin_make -DCMAKE_BUILD_TYPE="$1" --only-pkg-with-deps $_make_pkg
    cd "$OLDPWD"
}

alias bender_make_Release='_uch_make_common Release'
alias bender_make_RelWithDebInfo='_uch_make_common RelWithDebInfo'
alias bender_make_Debug='_uch_make_common Debug'
alias bender_make=bender_make_RelWithDebInfo


_uch_system_reset_ws ()
{
    local ws user_path
    ws="$1"
    user_path="$(pwd)"

    cd "$UCH_WS"/"$ws"/src
    rm -rf CMakeLists.txt
    catkin_init_workspace
    cd ..
    rm -rf build
    catkin_make
    . devel/setup.bash

    cd "$user_path"
}

robot_system_repair_ws_overlay ()
{
    local user_path

    if ! _uch_check_user_confirmation ; then
        return 0
    fi

    user_path=$(pwd)

    # ROS
    . /opt/ros/indigo/setup.bash

    # forks overlay
    robot_system_reset_ws "forks_ws"
 
    # base overlay
    robot_system_reset_ws "base_ws"

    # soft overlay
    robot_system_reset_ws "soft_ws"
    
    # high overlay
    robot_system_reset_ws "high_ws"

    cd "$user_path"
}


