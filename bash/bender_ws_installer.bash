#!/bin/bash
#
# ## installation instructions
# #
# # just run this script anywhere (anywhere but the same workspace path!)
# #
# $ chmod +x bender_framework_install.bash
# $ ./bender_framework_install
#
#

# this script is not ready yet!
exit 1

# TODO:
# - que hacer en fallo de clones
# - hooks
# - catkin_make al final
# - clonar submódulos sin opción --recursive, y utilizando el username provisto.

## ======================================================
## utilities
## ======================================================

_bender_installer_check_rosindigo ()
{
    # ros-base is installed?
    dpkg -s ros-indigo-ros-base >/dev/null 2>/dev/null
    local rc="$?"
    if [ "$rc" = "1" ]; then
        echo "ros-indigo-ros-base is not installed."
        return 1
    fi
    
    # ROS setup.bash exists?
    if [ ! -e /opt/ros/indigo/setup.bash ]; then
        echo "File not found: /opt/ros/indigo/setup.bash"
        return 1
    fi

    return 0
}


_bender_installer_reset_ws ()
{
    local ws_path user_path
    ws_path="$1"
    user_path="$(pwd)"

    mkdir -p "$ws_path"/src
    cd "$ws_path"/src
    rm -rf CMakeLists.txt
    catkin_init_workspace
    cd ..
    rm -rf build/
    rm -rf devel/
    rm -rf install/
    catkin_make
    source devel/setup.bash
    cd "$user_path"
}


## ======================================================
## PRE CHECKS
## ======================================================

# ROS is installed
if ! _bender_installer_check_rosindigo; then
    echo "Please, install ROS Indigo before proceeding."
    exit 1
fi


# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# select a folder were to install the framework.
# default=/home/user/bender_ws
framework_path="$HOME/bender_ws2"
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!
# SOLUCIONAR ESTO!!!!!!!!!!!!!!!!


while true; do

    read -e -p " - Select the framework path (default: $framework_path): " _user_path

    # empty response
    if [ -z "$_user_path" ]; then

        # path is not a directory
        if [ ! -e "$framework_path" ]; then
            mkdir -p "$framework_path"
        else

            if [ ! -d "$framework_path" ]; then
                printf "Sorry, $framework_path is a regular file, must be a directory."
                continue
            fi
        fi
        break
    fi

    # get a full path (if it exists)
    if [ -e "$_user_path" ]; then
        _user_path="$(readlink -f "$_user_path")"
    fi

    # path exists and is a directory
    if [ -d "$_user_path" ] ;then

        _n_files="$(find "$_user_path" -maxdepth 1 | wc -l)"
        if [ "$_n_files" = "1" ] ; then
            #echo "and is empty"
            framework_path="$_user_path"
            unset _n_files
            break
        fi
        unset _n_files

        # directory is not empty
        read -p " - $_user_path is not empty, do you want proceed anyway? [y/N]: " _answer
        if echo "$_answer" | grep -iq "^y" ;then
            framework_path="$_user_path"
            unset _answer
            break
        fi
        unset _answer
        continue
    fi

    # path is not a directory
    if [ -e "$_user_path" ]; then
        echo " - Sorry, but $_user_path is a file."
        continue
    fi

    # path does not exists
    read -p " - $_user_path does not exists, do you want to create this folder? [y/N]: " _answer
    if echo "$_answer" | grep -iq "^y" ;then
        mkdir -p "$_user_path"                      # create folder
        _user_path="$(readlink -f "$_user_path")"   # full path (after folder creation!)
        echo " - creating folder: $_user_path ..."
        framework_path="$_user_path"
        unset _answer
        break
    fi
    unset _answer

done
unset _user_path
echo "Using path: $framework_path"



## PREPARE WORKSPACE
## ========================================
# at this point:
# - framework_path contains the fullpath to a (existent) folder
# - framework_path could be a non empty directory
# - ROS indigo is installed under /opt/ros/indigo
#

## create workspaces and create the ROS overlay
## ---------------------------------------------

# # ROS baseline 
# source /opt/ros/indigo/setup.bash

# # forks_ws overlays ROS baseline
# _bender_installer_reset_ws "$framework_path"/forks_ws

# # base_ws overlays forks_ws
# _bender_installer_reset_ws "$framework_path"/base_ws

# # soft_ws overlays base_ws
# _bender_installer_reset_ws "$framework_path"/soft_ws

# # high_ws overlays soft_ws
# _bender_installer_reset_ws "$framework_path"/high_ws



## GET REPOSITORIES
## ========================================
#
# repos are not cloned again if they already exists

# bitbuket username
_username=""
_use_username=true

_bender_installer_get_repository ()
{
    local _clone_repo _repo_name _repo_path _repo_url _branchname
    _repo_path="$1"
    _repo_url="$2"
    _branchname="$3"

    # determine repository existence
    _clone_repo=true
    if [ -d "$_repo_path" ]; then
        echo "repo already exists"

        # .git exists
        if [ -e "$_repo_path"/.git ]; then
            echo ".git folder already exists "
            _clone_repo=false
        fi
    fi

    # clone if needed
    _repo_name="$(echo "$_repo_url" | sed 's/.*\///' | sed 's/.git//')"
    if $_clone_repo; then
        rm -rf "$_repo_path"

        # first username query
        if $_use_username && [ -z "$_username" ]; then
            
            printf "[Repository: %s]\n" "$_repo_name"
            printf " - If this computer will only be used by a single user, the git login\n"
            printf "process can be eased. Otherwise, you will have to provide your userneme\n"
            printf "for every fetch/pull/push command.\n"

            read -e -p  "Bitbucket username (or leave it empty): " _username
            _username="$(echo "$_username" | tr -d ' ')"

            if [ -z "$_username" ]; then
                _username=""
                _use_username=false
            else
                echo "Using username: $_username"
                _use_username=true
            fi
        fi

        # use previous username
        if [ -z "$_username" ]; then
            echo "Cloning repository '$_repo_name' into: $_repo_path."
        else
            echo "Cloning repository '$_repo_name' for user: '$_username' into: $_repo_path."
            _repo_url="$(echo "$_repo_url" | sed "s/bitbucket.org/$_username@bitbucket.org/")"
        fi
        git clone --recursive "$_repo_url" "$_repo_path"

        user_path=$(pwd)
        cd "$_repo_path"
        git checkout "$_branchname"
        git submodule foreach --recursive git checkout develop
        cd "$user_path"

    else
        echo "Repository '$_repo_name' already exists on $_repo_path. The clone will not be performed."
    fi
}

_bender_installer_get_repository_for_ws ()
{
    local _repo_path _repo_url _branchname
    _repo_path="$1"
    _repo_url="$2"
    _branchname="$3"

    # save CMakeLists.txt
    mv "$_repo_path"/CMakeLists.txt /tmp/bender_CMakeLists.txt

    # clone if necessary.. this would remove the src folder
    _bender_installer_get_repository "$_repo_path" "$_repo_url" "$_branchname"

    # recover CMakeLists.txt
    mv /tmp/bender_CMakeLists.txt "$_repo_path"/CMakeLists.txt
}


# bender_system
_repo_path="$framework_path"/bender_system
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_system.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "develop"

# base layer
_repo_path="$framework_path"/base_ws/src
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_base_layer
_bender_installer_get_repository_for_ws "$_repo_path" "$_repo_url" "develop"

# soft layer
_repo_path="$framework_path"/soft_ws/src
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_soft_layer
_bender_installer_get_repository_for_ws "$_repo_path" "$_repo_url" "develop"

# high layer
_repo_path="$framework_path"/high_ws/src
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_high_layer
_bender_installer_get_repository_for_ws "$_repo_path" "$_repo_url" "develop"

# bender_code_graveyard
_repo_path="$framework_path"/bender_code_graveyard
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_code_graveyard.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "develop"

# bender_embedded
_repo_path="$framework_path"/bender_embedded
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_embedded.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "develop"

# forks: rosaria
_repo_path="$framework_path"/forks_ws/src/rosaria
_repo_url=https://bitbucket.org/uchile-robotics-die/bender_fork_rosaria.git
_bender_installer_get_repository "$_repo_path" "$_repo_url" "master"



unset _use_username _username
unset framework_path

exit 1


## ENABLE HOOKS
## ==========================================


# # bender_system submodules
# hooks

# # soft layer submodules
# bender_tools
# bender_hri
# bender_manipulation
# bender_navigation
# bender_perception


# hooks




# copiar bender.sh a BENDER_WS/bender.sh
# modificar la variable BENDER_WS dentro de bender.sh
# avisar de hacer el source en ./bashrc a mano


# -------------------------------------------
#
# EMPEZAR OTRO SCRIPT AQUI... UNO QUE INSTALE TODAS LAS DEPENDENCIAS DE TODO
# USO DE BENDER_INSTALL
#

## INSTALL DEPENDENCIES
## ==========================================

# checkear si estan instalados los packetes necesarios



## COMPILE STUFF 
## ==========================================




