#!/bin/sh

###############################################################################
#   OVERVIEW
###############################################################################
# uch_find_string        - finds string within uch src space
# uch_cd                 - cd to a robot framework directory
# cdb                    - the same as uch_cd but faster to type. The "b"
#                          stands for Bender and legacy users.
# uch_printenv           - prints all UCH_* environment variables
# uch_refresh_shell      - reexecutes a shell to resource the robot framework
# uch_git_show_untracked - lists currently untracked files
# uch_git_show_ignored   - lists currently ignored files

###############################################################################
#   shell utilities
###############################################################################

# prevent failure
if _uch_check_if_bash_or_zsh ; then

    # prints all UCH_* environment variables and its values
    uch_printenv ()
    {
        printenv | sort | grep "UCH_.*=" 
    }

    ## uch_refresh_shell
    # executes bash to resource the robot framework.
    #
    # this is for testing purposes only!. Do not use it
    # when environment variables have changed. Open a new
    # terminal session instead.
    uch_refresh_shell ()
    {
        export UCH_FRAMEWORK_TWICE_LOAD_CHECK=false
        exec "$UCH_FRAMEWORK_LOADED_SHELL"
    }
    

    # lists currently untracked files
    uch_git_show_untracked ()
    {
        git ls-files --others
    }

    # lists currently ignored files
    uch_git_show_ignored ()
    {
        git check-ignore -v *
    }

    # the same as uch_cd but faster to type 
    alias cdb="uch_cd"
    alias bviz="roslaunch bender_utils rviz.launch" # TODO: move this 
    
fi


uch_open_config ()
{

    local _conf _editor
    _conf="$UCH_SHELL_CFG"

    # check file existence
    if [ ! -e "$_conf" ]; then
        printf "User configuration file not found: %s\n" "$_conf"
        return 1
    fi

    # check if EDITOR variable is unset
    _editor="$EDITOR"
    if [ -z "$_editor" ]; then
        printf "EDITOR env variable is unset. Using 'cat'\n"
        _editor="cat"
    fi
    if ! _uch_check_var_isset "EDITOR"; then
        printf "EDITOR env variable is unset. Using 'cat'\n"
        _editor="cat"
    fi
    printf " - EDITOR env variable resolves to: '%s'\n" "$_editor"

    # check editor is installed
    if ! _uch_check_installed "$_editor"; then
        printf "Sorry, but '%s' is not installed. Using 'cat'\n" "$_editor"
        _editor="cat"
    fi


    printf " - opening user config file '%s' with '%s'\n" "$_conf" "$_editor"    
    "$_editor" "$_conf" &
}

## uch_find_string
# see also: uch_find_string --help 
uch_find_string ()
{
    local string _path user_path opt show_help curpath

    _path="$UCH_SYSTEM"                # system
    _path="$_path $UCH_ROS_WS/base_ws/src" # base_ws
    _path="$_path $UCH_ROS_WS/soft_ws/src" # soft_ws
    _path="$_path $UCH_ROS_WS/high_ws/src" # high_ws

    string=""
    if [ "$#" = "2" ]; then 

        opt="$1"
        case "$opt" in

            "system"    ) _path="$UCH_SYSTEM" ;;
            "forks"     ) _path="$UCH_ROS_WS/forks_ws/src" ;;
            "base"      ) _path="$UCH_ROS_WS/base_ws/src" ;;
            "soft"      ) _path="$UCH_ROS_WS/soft_ws/src" ;;
            "high"      ) _path="$UCH_ROS_WS/high_ws/src" ;;
            "graveyard" ) _path="$UCH_GRAVEYARD" ;;
            "embedded"  ) _path="$UCH_EMBEDDED" ;;
            "all" )
                _path="$_path $UCH_EMBEDDED"
                _path="$_path $UCH_ROS_WS/forks_ws/src"
                ;;

            # unknown
            * ) 
                echo "Unknown option: $opt"
                show_help=true ;;
        esac
        string="$2"
    elif [ "$#" = "1" ]; then

        string="$1"
        case "$string" in
            "-h" | "--help" ) show_help=true ;;
            -* | --* )
                echo "Unknown option: $string"
                show_help=true ;;
            * ) ;;
        esac

    else
        show_help=true
    fi

    if [ -z "$string" ]; then
        echo "I won't lookup for an empty string!"
        show_help=true  
    fi

    if [ "$show_help" = true ]; then
        cat <<EOF
Synopsis:                
    uch_find_string [<workspace>] <string>

Description:
    It looks for instances of a <string> written on any file
    located on at least 1 robot workspace.

Options:
    Through the 'workspace' option you can change the location
    where lookup will be executed.

    Supported values are:
        - forks    : lookup on forks_ws
        - base     : lookup on base_ws
        - soft     : lookup on soft_ws
        - high     : lookup on high_ws
        - system   : lookup on system
        - graveyard: lookup on graveyard
        - embedded : lookup on embedded
        - all      : lookup on all previous locations, except graveyard

    By default the lookup is executed on system-base-soft-high.
EOF

        _uch_admin_goodbye
        return 1
    fi
    
    # i used this "cd" trick because we don't want fullpaths
    # displayed by grep, but shorter ones
    user_path=$(pwd)

    # parse the string array in a bash like manner
    if _uch_check_if_zsh ; then
        setopt local_options shwordsplit
    fi
    for curpath in $_path; do

        echo "$curpath"
        cd "$curpath"


        echo "[INFO]: Working on path: $curpath"
        # inside files
        echo "[INFO]: - Looking for pattern '$string' inside files:"
        grep -rInH --exclude-dir="\.git" "$string" .

        # in filenames 
        echo "[INFO]: - Looking for pattern '$string' in filenames:"
        find . -wholename "*$string*" -print -o -path "*/.git" -prune
    done

    cd "$user_path"

    return 0
}

## uch_cd
# see also: uch_cd --help
uch_cd ()
{
    local user_path show_help _path pkg_name pkg stack_name stack
    _path=""

    if _uch_check_if_zsh ; then
        setopt local_options shwordsplit
    fi
    
    user_path="$1"

    if [ "$#" = "0" ]; then
        _path="$UCH_ROS_WS"

    elif [ "$#" = "1" ]; then

        case "$user_path" in

            "system"    ) _path="$UCH_SYSTEM" ;;
            "forks"     ) _path="$UCH_ROS_WS/forks_ws/src" ;;
            "base"      ) _path="$UCH_ROS_WS/base_ws/src" ;;            
            "soft"      ) _path="$UCH_ROS_WS/soft_ws/src" ;;
            "high"      ) _path="$UCH_ROS_WS/high_ws/src" ;;
            "graveyard" ) _path="$UCH_GRAVEYARD" ;;
            "embedded"  ) _path="$UCH_EMBEDDED" ;;

             "-h" | "--help" ) show_help=true ;;

            -* | --* )
                echo "Unknown option: $user_path"
                show_help=true ;;

            # unknown --> package
            * )
                pkg_name="$user_path"
                for pkg in $UCH_PACKAGES
                do
                    if [ "$pkg" = "$pkg_name" ]; then
                        _path=$(rospack find "$pkg_name")
                        cd "$_path"
                        return 0
                    fi
                done


                stack_name="$user_path"
                for stack in $UCH_STACKS
                do
                    if [ "$stack" = "$stack_name" ]; then
                        _path=$(rosstack find "$stack_name")
                        cd "$_path"/..
                        return 0
                    fi
                done

                echo "Bender (meta)package named '$pkg_name' doesn't not exists. Try with 'roscd' command."
                show_help=true
        esac
    else
        show_help=true
    fi

    if [ "$show_help" = true ]; then
        cat <<EOF
Synopsis:                
    uch_cd [<workspace>|<package>|-h|--help]

Description:
    It changes the current directory to the root of the
    selected workspace or ROS (meta)package.

Options:
    Available workspace options are:
        - forks    : 'cd' to forks_ws
        - base     : 'cd' to base_ws
        - soft     : 'cd' to soft_ws
        - high     : 'cd' to high_ws
        - system   : 'cd' to system
        - graveyard: 'cd' to graveyard
        - embedded : 'cd' to embedded
    
    Available package options correspond to ROS (meta)packages named 'bender_*'.
    
    If no option is given, then the directory will be the one 
    addressed by the \$UCH_WS environment variable.

EOF
        _uch_admin_goodbye
        return 1
    fi

    cd "$_path"
    return 0
}


# Kill gazebo gently
uch_killgz ()
{
    # Kill controllers spawners
    rosnode kill /bender/controller_spawner 
    rosnode kill /bender/neck_controller_spawner
    # Kill Gazebo server
    killall gzserver
    # Kill Gazebo client
    killall gzclient
}

killgz ()
{
    # UCH_DEPRECATED : mark method as deprecated. The flag is
    # useful for looking up deprecated methods.

    echo "DEPRECATED ... Please call uch_killgz (gently)"
    echo "THIS METHOD WILL BE REMOVED FOR THE NEXT RELEASE"
    uch_killgz
}

uch_net_enable ()
{
    python "$UCH_SYSTEM"/shell/ros_network_indicator/ros_network_indicator.py --enable "$HOME"/bender.sh
    . "$HOME"/bender.sh
}

uch_net_disable ()
{
    python "$UCH_SYSTEM"/shell/ros_network_indicator/ros_network_indicator.py --disable "$HOME"/bender.sh
    . "$HOME"/bender.sh
}

