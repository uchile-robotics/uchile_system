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

# TODO:
# - mejorar manejo de fallo de clones
# - agregar manejo de fallo de update de submodulos
# - catkin_make al final

## ======================================================
## utilities
## ======================================================

_bender_installer_check_rosindigo ()
{
    # ros-base is installed?
    dpkg -s ros-indigo-ros-base >/dev/null 2>/dev/null
    local rc="$?"
    if [ "$rc" = "1" ]; then
        printf "ros-indigo-ros-base is not installed.\n"
        return 1
    fi
    
    # ROS setup.bash exists?
    if [ ! -e /opt/ros/indigo/setup.bash ]; then
        printf "File not found: /opt/ros/indigo/setup.bash \n"
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
    printf "Please, install ROS Indigo before proceeding.\n"
    exit 1
fi

# select a folder were to install the framework.
# default=/home/user/bender_ws
framework_path="$HOME/bender_ws"


while true; do

    read -e -p " - Select the framework path (default: $framework_path): " _user_path

    # empty response
    if [ -z "$_user_path" ]; then

        # path is not a directory
        if [ ! -e "$framework_path" ]; then
            mkdir -p "$framework_path"
        else

            if [ ! -d "$framework_path" ]; then
                printf "Sorry, %s is a regular file, must be a directory.\n" "$framework_path"
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
        printf " - Sorry, but %s is a file.\n" "$_user_path"
        continue
    fi

    # path does not exists
    read -p " - $_user_path does not exists, do you want to create this folder? [y/N]: " _answer
    if echo "$_answer" | grep -iq "^y" ;then
        mkdir -p "$_user_path"                      # create folder
        _user_path="$(readlink -f "$_user_path")"   # full path (after folder creation!)
        printf " - creating folder: %s ...\n" "$_user_path"
        framework_path="$_user_path"
        unset _answer
        break
    fi
    unset _answer

done
unset _user_path
printf "Using path: %s\n" "$framework_path"



## PREPARE WORKSPACE
## ========================================
# at this point:
# - framework_path contains the fullpath to a (existent) folder
# - framework_path could be a non empty directory
# - ROS indigo is installed under /opt/ros/indigo
#

## create workspaces and create the ROS overlay
## ---------------------------------------------

# ROS baseline 
source /opt/ros/indigo/setup.bash

# forks_ws overlays ROS baseline
_bender_installer_reset_ws "$framework_path"/forks_ws

# base_ws overlays forks_ws
_bender_installer_reset_ws "$framework_path"/base_ws

# soft_ws overlays base_ws
_bender_installer_reset_ws "$framework_path"/soft_ws

# high_ws overlays soft_ws
_bender_installer_reset_ws "$framework_path"/high_ws



## GET REPOSITORIES
## ========================================
#
# repos are not cloned again if they already exists

# bitbuket username
_username=""
_use_username=true

_bender_installer_get_repository ()
{
    local _clone_repo _repo_name _repo_path _repo_url _branchname _gitmodules
    local _user_path
    _repo_path="$1"
    _repo_url="$2"
    _branchname="$3"

    # determine repository existence
    _clone_repo=true
    if [ -d "$_repo_path" ]; then
        printf "repo already exists\n"

        # .git exists
        if [ -e "$_repo_path"/.git ]; then
            printf ".git folder already exists\n"
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
                printf "Using username: %s\n" "$_username"
                _use_username=true
            fi
        fi

        # clone using previous username
        if [ -z "$_username" ]; then
            printf "Cloning repository '%s' into: %s.\n" "$_repo_name" "$_repo_path"
        else
            printf "Cloning repository '%s' for user: '%s' into: %s.\n" "$_repo_name" "$_username" "$_repo_path"
            _repo_url="$(echo "$_repo_url" | sed "s/bitbucket.org/$_username@bitbucket.org/")"
        fi
        git clone "$_repo_url" "$_repo_path"

        # clean-up failed clone
        local _rc="$?"
        if [ "$_rc" = "128" ] || [ ! -d "$_repo_path" ] ; then
            printf "\n"
            printf "UPS.. The clone process failed for: %s\n" "$_repo_url"
            printf "I recommend you to run this script again\n"
            printf "\n"
            rm -rf "$_repo_path"
            return 1
        fi

        # do checkout
        _user_path=$(pwd)
        cd "$_repo_path"
        git checkout "$_branchname"


        # check submodules
        _gitmodules="$_repo_path"/.gitmodules
        if [ ! -e "$_gitmodules" ]; then
            cd "$_user_path"
            return 0
        fi

        # modify modules
        if [ ! -z "$_username" ]; then
            # modify .gitmodules
            sed --in-place=_bkp "s/\/\/bitbucket.org/\/\/$_username@bitbucket.org/" "$_gitmodules"
        fi
        
        # update
        git submodule init
        git submodule update
        git submodule foreach --recursive git checkout "$_branchname"

        # restore bkp
        if [ ! -z "$_username" ]; then
            mv "$_gitmodules"_bkp "$_gitmodules"
        fi

        cd "$_user_path"

    else
        printf "Repository '%s' already exists on %s. The clone will not be performed.\n" "$_repo_name" "$_repo_path"
    fi
    return 0
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
    local _rc="$?"
    
    # recover CMakeLists.txt
    mv /tmp/bender_CMakeLists.txt "$_repo_path"/CMakeLists.txt

    if [ "$_rc" = "1" ]; then 
        exit 1
    fi
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


## ENABLE HOOKS
## ==========================================
_hook_file="$framework_path"/bender_system/hooks/hooks/pre-commit

_bender_installer_enable_hook ()
{
    local _new_hookfile
    _new_hookfile="$1"

    printf "Installing githook on path: %s/pre-commit, from original %s\n" "$_new_hookfile" "$_hook_file"

    if [ ! -d "$_new_hookfile" ]; then
        printf "Folder not found: %s\n" "$_new_hookfile"

        local count="$(printf "%s\n" "$_new_hookfile" | grep ".git/modules/" | wc -c)"
        if [ ! "$count" = "0" ]; then
            printf "This is a uninitialized submodule.\n"
            printf "Try:\n > git submodule init\n > git submodule update\n\n"
        fi
        return 1
    fi

    _new_hookfile="$_new_hookfile"/pre-commit
    cp "$_hook_file" "$_new_hookfile"
    chmod 775 "$_new_hookfile"

    return 0
}


## bender_system
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/bender_system/.git/hooks

# submodule: hook
_bender_installer_enable_hook "$framework_path"/bender_system/.git/modules/hooks/hooks


## bender_base_layer
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/base_ws/src/.git/hooks


## bender_soft_layer
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/hooks

# submodule: bender_hri
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_hri/hooks

# submodule: bender_tools
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_tools/hooks

# submodule: bender_manipulation
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_manipulation/hooks

# submodule: bender_navigation
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_navigation/hooks

# submodule: bender_perception
_bender_installer_enable_hook "$framework_path"/soft_ws/src/.git/modules/bender_perception/hooks


## bender_high_layer
## ---------------------------------------------------
# repo
_bender_installer_enable_hook "$framework_path"/high_ws/src/.git/hooks



## bender_code_graveyard
## ---------------------------------------------------
_bender_installer_enable_hook "$framework_path"/bender_code_graveyard/.git/hooks

## bender_embedded
## ---------------------------------------------------
_bender_installer_enable_hook "$framework_path"/bender_embedded/.git/hooks

## forks: rosaria
## ---------------------------------------------------
_bender_installer_enable_hook "$framework_path"/forks_ws/src/rosaria/.git/hooks

unset _hook_file


## BENDER.SH SOURCING
## ==========================================

# bender.sh
# -----------------------------

# prepare file
template="$framework_path"/bender_system/templates/bender.sh
sed --in-place=_bkp 's,"$HOME"/bender_ws,'"$framework_path"',' "$template"

# copy it
cp -f "$template" "$HOME"/bender.sh

# restore it
mv "$template"_bkp "$template"
unset template

# .bashrc
# ----------------------------

printf "The installation is almost ready. To finish, just source the %s script onto your .bashrc file.\n" "$HOME"/bender.sh
printf "e.g:\n"
printf " > echo 'source \"\$HOME\"/bender.sh' >> .bashrc\n"
printf "\n"

unset framework_path
