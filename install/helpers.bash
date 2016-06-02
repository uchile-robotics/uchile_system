#!/bin/bash

#
# checks whether ROS INDIGO baseline is installed or no
_bender_installer_check_rosindigo ()
{
    

    # ros-base is installed?
    dpkg -s ros-indigo-ros-base >/dev/null 2>/dev/null
    local rc="$?"
    if [ "$rc" = "1" ]; then
        printf "ros-indigo-ros-base is not installed.\n"
        return 1
    fi
    printf " - ROS indigo is installed (ros-indigo-ros-base)\n"
    
    # ROS setup.bash exists?
    if [ ! -e /opt/ros/indigo/setup.bash ]; then
        printf "File not found: /opt/ros/indigo/setup.bash \n"
        return 1
    fi
    printf " - indigo setup.bash exists\n"

    return 0
}

#
# promts the user for a VALID location for the framework
# - if the location does not exists, then the user decides about its creation
# - otherwise, the path must be a writtable folder
#   if this folder is not empty, the user decides if the script continues or not.
#
# it creates the framework_path var
_bender_installer_ask_framework_path ()
{
    local _default_path _user_path _n_files _answer
    framework_path=""

    # default
    _default_path="$1"

    while true; do

        # ask
        read -e -p " - Select the framework path (default: $_default_path): " _user_path

        # handle default response: empty --> default
        if [ -z "$_user_path" ]; then
            _user_path="$_default_path"
        fi

        # "full" path
        case "$_user_path" in
            /*) _user_path="$_user_path";;
            *)  _user_path="$PWD/$_user_path";;
        esac

        # path is an existent directory
        if [ -d "$_user_path" ]; then

            # check writtable dir
            if [ ! -w "$_user_path" ] ; then
                echo "You do not have write permissions on this location!"
                continue
            fi

            # check emptyness
            _n_files="$(find "$_user_path" -maxdepth 1 | wc -l)"

            # dir is empty
            if [ "$_n_files" = "1" ] ; then
                #echo "and is empty"
                framework_path="$_user_path"
                break
            fi

            # directory is not empty
            read -p " - $_user_path is not empty, do you want proceed anyway? [y/N]: " _answer
            if echo "$_answer" | grep -iq "^y" ;then
                framework_path="$_user_path"
                break
            fi
            continue

        fi

        # path is an existent file
        if [ -e "$_user_path" ]; then
            printf " - Sorry, but %s is a file.\n" "$_user_path"
            continue
        fi
        
        # path does not exists --> create it?
        read -p " - $_user_path does not exists, do you want to create this folder? [y/N]: " _answer
        if echo "$_answer" | grep -iq "^y" ; then

            # attemp to create folder
            mkdir -p "$_user_path" >/dev/null 2>/dev/null
            local rc="$?"
            if [ "$rc" = "1" ]; then
                printf "Failed to create directory. Maybe you do not have the permissions.\n"
                continue
            fi

            # creation succeeded
            _user_path="$(readlink -f "$_user_path")"   # full path (after folder creation!)
            printf " - creating folder: %s ...\n" "$_user_path"
            framework_path="$_user_path"
            break
        fi
    done
}

#
# prompts the user for an username
_bender_installer_ask_username ()
{
    local  _username
    read -e -p " - Bitbucket username: " _username
    echo "$_username"
}

#
# prompts the user for a password
_bender_installer_ask_password ()
{
    local _password
    read -es -p " - Bitbucket password: " _password
    echo "$_password"
}

#
# prompts the user for an username
# - writes variables username and password
# - this vars will be empty if the response was a negation
_bender_installer_ask_single_username ()
{
    local _answer
    username=""
    password=""

    printf " - If this computer will only be used by a single user, the git login\n"
    printf "   process can be eased. Otherwise, you will have to provide your username\n"
    printf "   for every fetch/pull/push command.\n"
    printf "\n"
    printf " - (hints time!):\n"
    printf "   + it is highly recommended to provide your repository credentials if you\n"
    printf "     will be the only one using this account\n"
    printf "   + if this is a shared account, then using this feature will be a headache!\n"
    printf "     obs: NOW you will be prompted for the credentials of each repo!\n"
    printf "\n"

    read -p " - So, do you want to setup your credentials? [y/N]: " _answer
    if echo "$_answer" | grep -iq "^y" ; then        
        
        printf " - OK, i need the following data:\n"
        username=$(_bender_installer_ask_username)
        password=$(_bender_installer_ask_password)
        printf "\n"
        #echo "$username"
        #echo "$password"
    fi
}

#
# resets completely a workspace!
# - anything roslike is deleted, but src/
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

# cleans password occurrences on a git url for the given file.
# - only works on repositories were the username is set up
# - requires the filename: e.g: .git/config, .git/logs/HEAD
# - requires the asociated username: myuser
_bender_intaller_clean_url_pass_from_file () {

    local _username _filename
    _username="$1"
    _filename="$2"

    # check arguments
    if [ -z "$1" ]; then
        printf " - [ERROR]: _bender_installer_clean_repo_password requires 2 arguments: _username _filename\n"
        return 1
    fi
    if [ -z "$2" ]; then
        printf " - [ERROR]: _bender_installer_clean_repo_password requires 2 arguments: _username _filename\n"
        return 1
    fi

    # check file existence
    if [ ! -e "$_filename" ]; then
        printf " - [ERROR]: attemped to clean a nonexistent file: %s\n" "$_filename"
        return 1
    fi

    #echo " - [DEBUG]: cleaning file: $_filename for user $_username"
    sed --in-place "s/$_username:.*@/$_username@/" "$_filename"
    return 0
}

# cleans password occurrences from .git module folder, for the given username
# - only works on repositories were the username is set up
# - requires the module path. e.g: .git/module/my_module
# - requires the asociated username: myuser
_bender_intaller_clean_url_pass_from_module () {

    local _username _modulepath _headspath
    _username="$1"
    _modulepath="$2"

    # check arguments
    if [ -z "$1" ]; then
        printf " - [ERROR]: _bender_installer_clean_repo_password requires 2 arguments: _username and _modulepath\n"
        return 1
    fi
    if [ -z "$2" ]; then
        printf " - [ERROR]: _bender_installer_clean_repo_password requires 2 arguments: _username and _modulepath\n"
        return 1
    fi

    # check directory existence
    if [ ! -d "$_modulepath" ]; then
        printf " - [ERROR]: attemped to clean a nonexistent module directory\n"
        return 1
    fi

    #echo " - [DEBUG]: cleaning module on path: $_modulepath for user $_username"

    # clean repo reference + 1 line for each submodule
    _bender_intaller_clean_url_pass_from_file "$_username" "$_modulepath/config"
    _bender_intaller_clean_url_pass_from_file "$_username" "$_modulepath/logs/HEAD"
    _bender_intaller_clean_url_pass_from_file "$_username" "$_modulepath/logs/refs/remotes/origin/HEAD"

    _headspath="$_modulepath/logs/refs/heads"
    if [ ! -d "$_headspath" ]; then
        return 0
    fi

    for _branch in $(ls $_headspath)
    do
        #echo " - [DEBUG]: branch: $_branch"
        _bender_intaller_clean_url_pass_from_file "$_username" "$_headspath/$_branch"
    done

    return 0
}

# cleans password occurrences from .git folder, for the given username
# - only works on repositories were the username is set up
# - assumes it is located in a repository. e.g: echo $pwd  --> "~/my_repo/"
# - requires the asociated username
_bender_installer_clean_repo_password () {

    local _username _module
    _username="$1"

    if [ -z "$1" ]; then
        printf " - [ERROR]: _bender_installer_clean_repo_password requires 1 argument: _username\n"
        return 1
    fi

    # check repository existence
    if [ ! -e ".git/" ]; then
        printf " - [ERROR]: attemped to clean repository, but current path does not points to one\n"
        return 1
    fi

    # clean repo references
    _bender_intaller_clean_url_pass_from_module "$_username" ".git"
    
    # no modules, then return
    if [ ! -d ".git/modules" ]; then
        return 0
    fi

    # clean modules
    for _module in $(ls .git/modules)
    do
        _bender_intaller_clean_url_pass_from_module "$_username" ".git/modules/$_module"
    done

    return 0
}

## clones a repository from the given location
# requires the following:
# - [1] _repo_path : where to clone the repo
# - [2] _repo_url  : the repo url
# - [3] _use_credentials: provides username and pass?
# - [4] _username : the username
# - [5] _password : the user password
_bender_installer_get_repository ()
{
    local _repo_name _repo_path _repo_url _username _password _use_credentials
    local _user_path _show_user _gitmodules
    _repo_path="$1"
    _repo_url="$2"
    _use_credentials="$3"
    _username="$4"
    _password="$5"


    # presentation
    _show_user="$_username"
    if [ "$use_credentials" = "false" ]; then
        _show_user=" -- not given -- "
    fi
    _repo_name="$(echo "$_repo_url" | sed 's/.*\///' | sed 's/.git//')"
    printf "\n - - - - - - - \n"
    printf " - repository name: $_repo_name\n"
    printf " - repository url : $_repo_url\n"
    printf " - destiny path   : $_repo_path\n"
    printf " - user name      : $_show_user\n"


    # check repository existence
    if [ -e "$_repo_path"/.git ]; then
        printf " - .git folder already exists\n"        
        printf " - won't clone again. A repository already exists here!\n"
        return 0
    fi
    rm -rf "$_repo_path" # delete if existing but is not a repository

 
    # clone using previous username
    _repo_url_clean="$_repo_url"
    if [ "$use_credentials" = "true" ]; then
        _repo_url="$(echo "$_repo_url" | sed "s/bitbucket.org/$_username:$_password@bitbucket.org/")"
    fi
    printf " - - - > \n"
    git clone "$_repo_url" "$_repo_path"

    # clean-up failed clone
    local _rc="$?"
    if [ "$_rc" = "128" ] || [ ! -d "$_repo_path" ] ; then
        printf "\n"
        printf "UPS.. The clone process failed for: %s\n" "$_repo_url_clean"
        printf "Maybe you should run this script again and mind your credentials!\n"
        printf "\n"
        rm -rf "$_repo_path"
        printf "<---\n"
        return 1
    fi
    printf "< - - - \n\n"
    printf " - clone OK\n"

    ## update submodules
    ## ------------------------------
    _user_path=$(pwd)
    cd "$_repo_path"

    # check submodules
    _gitmodules="$_repo_path"/.gitmodules
    if [ ! -e "$_gitmodules" ]; then
        printf " - no submodules were found for this repo\n"
        if [ "$use_credentials" = "true" ]; then
            _bender_installer_clean_repo_password "$_username"
        fi
        cd "$_user_path"
        return 0
    fi
    printf " - found some submodules.\n"
    printf " - now, i will attempt to update the submodules using the same credentials!\n"

    # modify .gitmodules
    printf " - (creating .gitmodules backup)\n"
    if [ ! -z "$_username" ]; then
        # modify .gitmodules
        sed --in-place=_bkp "s/\/\/@bitbucket.org/\/\/$_username:$_password@bitbucket.org/" "$_gitmodules"
    fi
    
    # update
    printf " - (git submodule init)\n"
    printf " - - - > \n"
    # --quiet: we dont want to display the user password from the repo url!
    git submodule --quiet init
    printf "< - - - \n\n"
    printf " - (git submodule update)\n"
    printf " - - - > \n"
    git submodule update
    printf "< - - - \n\n"

    # restore .gitmodules bkp
    printf " - (restoring .gitmodules backup)\n"
    if [ ! -z "$_username" ]; then
        mv "$_gitmodules"_bkp "$_gitmodules"
    fi

    # clean git state
    if [ "$use_credentials" = "true" ]; then
        _bender_installer_clean_repo_password "$_username"
    fi
    cd "$_user_path"

    return 0
}

## clones a repository from the given location
# requires the following:
# - [1] _repo_path : where to clone the repo
# - [2] _repo_url  : the repo url
# - [3] _use_credentials: provides username and pass?
# - [4] _username : the username
# - [5] _password : the user password
_bender_installer_get_repository_for_ws ()
{
    local _repo_name _repo_path _repo_url _username _password _use_credentials
    _repo_path="$1"
    _repo_url="$2"
    _use_credentials="$3"
    _username="$4"
    _password="$5"


    # save CMakeLists.txt
    mv "$_repo_path"/CMakeLists.txt /tmp/bender_CMakeLists.txt

    # clone if necessary.. this would remove the src folder
    _bender_installer_get_repository "$_repo_path" "$_repo_url" "$_use_credentials" "$_username" "$_password" 
    local _rc="$?"
    
    # recover CMakeLists.txt
    mv /tmp/bender_CMakeLists.txt "$_repo_path"/CMakeLists.txt

    if [ "$_rc" = "1" ]; then 
        return 1
    fi
}


_bender_installer_enable_hook ()
{
    local _new_hookfile
    _new_hookfile="$1"

    printf " - installing git hook:\n"
    printf "     - at  : %s/pre-commit" "$_new_hookfile"
    printf "     - from: %s\n"          "$_hook_file"

    if [ ! -d "$_new_hookfile" ]; then
        printf " - folder not found: %s\n" "$_new_hookfile"

        local count="$(printf "%s\n" "$_new_hookfile" | grep ".git/modules/" | wc -c)"
        if [ ! "$count" = "0" ]; then
            printf " - this is a uninitialized submodule.\n"
            printf "   try:\n > git submodule init\n > git submodule update\n\n"
        fi
        return 1
    fi

    _new_hookfile="$_new_hookfile"/pre-commit
    cp "$_hook_file" "$_new_hookfile"
    chmod 775 "$_new_hookfile"

    return 0
}

