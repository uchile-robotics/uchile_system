#!/bin/sh

##############################################################################################
#   CATKIN MAKE HELPER FUNCTIONS
##############################################################################################

_bender_make_common() {

    cd "$BENDER_WORKSPACE"/..
    _make_pkg="$2"

    if [ -z "$2" ]; then
        # build all packages by default
        _make_pkg=$BENDER_PACKAGES

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
        _make_pkg=$BENDER_PACKAGES

        catkin_make -DCMAKE_BUILD_TYPE="$1" --only-pkg-with-deps $_make_pkg --force-cmake -G"Eclipse CDT4 - Unix Makefiles"
        cd "$OLDPWD"
        return
    fi

    catkin_make -DCMAKE_BUILD_TYPE="$1" --only-pkg-with-deps $_make_pkg
    cd "$OLDPWD"
}

alias bender_make_Release='_bender_make_common Release'
alias bender_make_RelWithDebInfo='_bender_make_common RelWithDebInfo'
alias bender_make_Debug='_bender_make_common Debug'
alias bender_make=bender_make_RelWithDebInfo


_bender_system_reset_ws ()
{
    local ws user_path
    ws="$1"
    user_path="$(pwd)"

    cd "$BENDER_WS"/"$ws"/src
    rm -rf CMakeLists.txt
    catkin_init_workspace
    cd ..
    rm -rf build
    catkin_make
    . devel/setup.bash

    cd "$user_path"
}

bender_system_repair_ws_overlay ()
{
    local user_path
    user_path=$(pwd)

    # ROS
    . /opt/ros/indigo/setup.bash

    # forks overlay
    bender_system_reset_ws "forks_ws"
 
    # base overlay
    bender_system_reset_ws "base_ws"

    # soft overlay
    bender_system_reset_ws "soft_ws"
    
    # high overlay
    bender_system_reset_ws "high_ws"

    cd "$user_path"
}
