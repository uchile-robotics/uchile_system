#!/bin/bash


# checks whether ROS INDIGO baseline is installed or no
_uchile_check_rosindigo ()
{
    # ros-base is installed?
    dpkg -s ros-indigo-ros-base >/dev/null 2>/dev/null
    local rc="$?"
    if [ "$rc" = "1" ]; then
        printf " - [FAIL] ros-indigo-ros-base is not installed.\n"
        return 1
    fi
    printf " - [OK] ROS indigo is installed (ros-indigo-ros-base)\n"
    
    # ROS setup.bash exists?
    if [ ! -e /opt/ros/indigo/setup.bash ]; then
        printf " - [FAIL] File not found: /opt/ros/indigo/setup.bash \n"
        return 1
    fi
    printf " - [OK] indigo setup.bash exists\n"

    return 0
}

#
# prompts the user for a VALID location for the framework
# - if the location does not exists, then the user decides about its creation
# - otherwise, the path must be a writtable folder
#   if this folder is not empty, the user decides if the script continues or not.
#
# it creates the framework_path var
_uchile_ask_framework_path ()
{
    local _default_path _user_path _n_files _answer
    export framework_path=""

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
                export framework_path="$_user_path"
                break
            fi

            # directory is not empty
            read -p " - $_user_path is not empty, do you want proceed anyway? [y/N]: " _answer
            if echo "$_answer" | grep -iq "^y" ;then
                export framework_path="$_user_path"
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
            export framework_path="$_user_path"
            break
        fi
    done
}

# calls _uchile_create_complete_ws_handler on an isolated env
_uchile_create_complete_ws ()
{
    local ws_path
    ws_path="$1"

    # the "env -i bash --rcfile /etc/bash.bashrc" is required to run bash in a 
    # clean environment, where ROS is not sourced.
    env -i bash --rcfile /etc/bash.bashrc -c "source ${BASH_SOURCE}; _uchile_create_complete_ws_handler ${ws_path}"
}


# this requires ROS not to be sourced!
_uchile_create_complete_ws_handler ()
{
    local ws_path
    ws_path="$1"

    printf " - Building overlayed ROS workspaces at %s. (Delete CMakeLists.txt files to force rebuilds)\n" "$ws_path"

    # ROS sourcing
    source /opt/ros/indigo/setup.bash

    # forks_ws overlays ROS baseline
    _uchile_create_ws "$ws_path"/forks_ws

    # base_ws overlays forks_ws
    _uchile_create_ws "$ws_path"/base_ws

    # soft_ws overlays base_ws
    _uchile_create_ws "$ws_path"/soft_ws

    # high_ws overlays soft_ws
    _uchile_create_ws "$ws_path"/high_ws

    printf " ... built ROS overlays: ROS_PACKAGE_PATH=%s.\n\n\n" "$ROS_PACKAGE_PATH"
}


_uchile_create_ws ()
{
    local ws_path user_path
    ws_path="$1"
    user_path="$(pwd)"

    mkdir -p "$ws_path"/src
    cd "$ws_path"/src

    if [ -e CMakeLists.txt ]; then
        printf " - ... workspace at %s already exists.\n" "$ws_path"
        source ../devel/setup.bash
        return 0
    fi

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

# # resets completely a workspace!
# # - anything roslike excepting src/ is deleted 
# _uchile_reset_ws ()
# {
#     local ws_path user_path
#     ws_path="$1"
#     user_path="$(pwd)"

#     mkdir -p "$ws_path"/src
#     cd "$ws_path"/src
#     rm -rf CMakeLists.txt
#     catkin_init_workspace
#     cd ..
#     rm -rf build/
#     rm -rf devel/
#     rm -rf install/
#     catkin_make
#     source devel/setup.bash
#     cd "$user_path"
# }


## clones a repository from the given location
# requires the following:
# - [1] _repo_path : where to clone the repo
# - [2] _repo_url  : the repo url
_uchile_get_repository ()
{
    local _repo_name _repo_path _repo_url _branch
    local _user_path
    _repo_path="$1"
    _repo_url="$2"
    _branch="$3"
    _user_path=$(pwd)

    _repo_name="$(echo "$_repo_url" | sed 's/.*\///' | sed 's/.git//')"
    printf "\n - - - - - - - \n"
    printf " - repository name: %s\n" "$_repo_name"
    printf " - repository url : %s\n" "$_repo_url"
    printf " - destiny path   : %s\n" "$_repo_path"

    # check repository existence
    if [ -e "$_repo_path"/.git ]; then
        printf " - Found .git folder. Won't clone again.\n"
        return 0
    fi
    rm -rf "$_repo_path" # delete if existing but is not a repository


    # clone using previous username
    printf " - - - > \n"
    git clone "$_repo_url" "$_repo_path"

    # clean-up failed clone
    local _rc="$?"
    if [ "$_rc" = "128" ] || [ ! -d "$_repo_path" ] ; then
        printf "\n"
        printf "UPS.. The clone process failed for: %s\n" "$_repo_url"
        printf "Maybe you should run this script again!\n"
        printf "\n"
        rm -rf "$_repo_path"
        printf "<---\n"
        return 1
    fi
    printf " - clone OK\n"

    if [ ! -z "$_branch" ]; then
        printf " - checking out to hash: %s\n" "$_branch"
        cd "$_repo_path"
        git checkout "$_branch"
        cd "$_user_path"
    fi

    printf "< - - - \n"


    ## update submodules
    ## ------------------------------

    # check submodules
    if [ ! -e "$_repo_path"/.gitmodules ]; then
        printf " - no submodules were found for this repo\n"
        return 0
    fi
    printf " - found some submodules.\n"
    printf " - now, i will attempt to update the submodules.\n"


    cd "$_repo_path"

    # update
    printf " ---------------------- >>>> \n"
    printf " - (git submodule init)\n"
    git submodule init
    printf "<<<< ---------------------- \n\n"

    printf " ---------------------- >>>> \n"
    printf " - (git submodule update)\n"
    git submodule update
    printf "<<<< ---------------------- \n\n"

    cd "$_user_path"
    return 0
}


## clones a repository from the given location
# requires the following:
# - [1] _repo_path : where to clone the repo
# - [2] _repo_url  : the repo url
_uchile_get_repository_for_ws ()
{
    local _repo_name _repo_path _repo_url
    _repo_path="$1"
    _repo_url="$2"

    # save CMakeLists.txt
    if [ -e "$_repo_path"/CMakeLists.txt ]; then
        mv "$_repo_path"/CMakeLists.txt /tmp/uch_CMakeLists.txt
    fi

    # clone if necessary.. this would remove the src folder
    _uchile_get_repository "$_repo_path" "$_repo_url"
    local _rc="$?"
    
    # recover CMakeLists.txt
    if [ -e /tmp/uch_CMakeLists.txt ]; then
        mv /tmp/uch_CMakeLists.txt "$_repo_path"/CMakeLists.txt
    fi

    if [ "$_rc" = "1" ]; then 
        return 1
    fi
}


_uchile_enable_githook ()
{
    local _repo _template _hook_file _submodule
    _repo="$1"
    _template="$2"

    # checks
    if [ ! -d "$_repo/.git" ]; then
        printf "Path %s is not the root of a valid git repository.\n" "$_repo"
        return 1
    fi
    if [ ! -e "$_template" ]; then
        printf "Git hook template file not found at: '%s' .\n" "$_template"
        return 1
    fi
    _hook_file="$_repo/.git/hooks/pre-commit"
    if [ -e "$_hook_file" ]; then
        printf " - (re)installing git hook on repo '%s' from template '%s'\n" "$_repo" "$_template"
        rm -f "$_hook_file"
    else
        printf " - installing git hook on repo '%s' from template '%s'\n" "$_repo" "$_template"
    fi

    # install root hookfile
    mkdir -p "$_repo/.git/hooks"
    cp "$_template" "$_hook_file"
    chmod 775 "$_hook_file"

    # install on submodules
    if [ -d "$_repo/.git/modules" ]; then
        printf "    - looking for submodules\n"
        for _submodule in $(find "$_repo/.git/modules" -mindepth 1 -maxdepth 1 -type d)
        do
            printf "    - ... installing git hook on submodule '%s'\n" "$_submodule"
            _hook_file="$_submodule/hooks/pre-commit"
            mkdir -p "$_submodule/hooks"
            cp "$_template" "$_hook_file"
            chmod 775 "$_hook_file"
        done

    fi
    return 0
}

# requires the framework_path env variable to be set
function _uchile_link_ ()
{
    local target dest full_target full_dest
    target="$1"
    dest="$2"

    full_target="$framework_path/pkgs/$target"
    full_dest="$framework_path/ros/$dest"

    # target exists
    if [ ! -e "$full_target" ]; then
        printf "    - target not found: %s\n" "$full_target"
        return 1
    fi

    # destination existence
    if [ -e "$full_dest" ]; then
        printf "    - (re)creating link to destination: %s\n" "$dest"
        rm -f "$full_dest"
    else
        printf "    - creating link to destination: %s\n" "$dest"
    fi

    # symbolink link
    ln -sf "$full_target" "$full_dest"

    return 0
}
