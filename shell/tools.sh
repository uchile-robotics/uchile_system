#!/bin/sh

##############################################################################################
#   OVERVIEW
##############################################################################################
# bender_find_string        - finds string within bender src space
# bender_cd                 - cd to a bender framework directory
# cdb                       - the same as bender_cd but faster to type 
# bender_printenv           - prints all BENDER_* environment variables
# bender_refresh_shell      - reexecutes a shell to resource the bender framework
# bender_git_show_untracked - lists currently untracked files
# bender_git_show_ignored   - lists currently ignored files

##############################################################################################
#   shell utilities
##############################################################################################

# prevent failure
if _bender_check_if_bash_or_zsh ; then

    # prints all BENDER_* environment variables and its values
    function bender_printenv
    {
        printenv | sort | grep "BENDER_.*=" 
    }

    ## bender_refresh_shell
    # executes bash to resource the bender framework.
    #
    # this is for testing purposes only!. Do not use it
    # when environment variables have changed. Open a new
    # terminal session instead.
    function bender_refresh_shell
    {
        exec "$SHELL"
    }
    

    # lists currently untracked files
    function bender_git_show_untracked
    {
        git ls-files --others
    }

    # lists currently ignored files
    function bender_git_show_ignored
    {
        git check-ignore -v *
    }

    # the same as bender_cd but faster to type 
    alias cdb="bender_cd"
    alias bviz="roslaunch bender_utils rviz.launch"
    
fi


function bender_open_config {

    local _conf _editor
    _conf="$BENDER_SHELL_CFG"

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
    if ! _bender_check_var_isset "EDITOR"; then
        printf "EDITOR env variable is unset. Using 'cat'\n"
        _editor="cat"
    fi
    printf " - EDITOR env variable resolves to: '%s'\n" "$_editor"

    # check editor is installed
    if ! _bender_check_installed "$_editor"; then
        printf "Sorry, but '%s' is not installed. Using 'cat'\n" "$_editor"
        _editor="cat"
    fi


    printf " - opening user config file '%s' with '%s'\n" "$_conf" "$_editor"    
    "$_editor" "$_conf" &
}

## bender_find_string
# see also: bender_find_string --help 
bender_find_string ()
{
    local string _path user_path opt show_help curpath

    _path="$BENDER_SYSTEM"         # bender_system
    _path="$_path $BENDER_WS/base_ws/src" # base_ws
    _path="$_path $BENDER_WS/soft_ws/src" # soft_ws
    _path="$_path $BENDER_WS/high_ws/src" # high_ws

    string=""
    if [ "$#" = "2" ]; then 

        opt="$1"
        case "$opt" in

            "system"    ) _path="$BENDER_SYSTEM" ;;
            "base"      ) _path="$BENDER_WS/base_ws/src" ;;
            "soft"      ) _path="$BENDER_WS/soft_ws/src" ;;
            "high"      ) _path="$BENDER_WS/high_ws/src" ;;
            "graveyard" ) _path="$BENDER_WS/bender_code_graveyard" ;;
            "forks"     ) _path="$BENDER_WS/forks_ws/src" ;;
            "embedded"  ) _path="$BENDER_WS/bender_embedded" ;;
            "all" )
                _path="$_path $BENDER_WS/bender_embedded"
                _path="$_path $BENDER_WS/forks_ws/src"
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
    bender_find_string [<workspace>] <string>

Description:
    It looks for instances of a <string> written on any file
    located on at least 1 bender workspace.

Options:
    Through the 'workspace' option you can change the location
    where lookup will be executed.

    Supported values are:
        - base     : lookup on base_ws
        - soft     : lookup on soft_ws
        - high     : lookup on high_ws
        - system   : lookup on bender_system
        - graveyard: lookup on bender_code_graveyard
        - forks    : lookup on forks_ws
        - embedded : lookup on bender_embedded
        - all      : lookup on all previous locations, except graveyard

    By default the lookup is executed on system-base-soft-high.
EOF

        _bender_admin_goodbye
        return 1
    fi
    
    # i used this "cd" trick because we don't want fullpaths
    # displayed by grep, but shorter ones
    user_path=$(pwd)

    # parse the string array in a bash like manner
    if _bender_check_if_zsh ; then
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

## bender_cd
# see also: bender_cd --help
function bender_cd
{
    local user_path show_help _path pkg_name pkg stack_name stack
    _path=""

    if _bender_check_if_zsh ; then
        setopt local_options shwordsplit
    fi
    
    user_path="$1"

    if [ "$#" = "0" ]; then
        _path="$BENDER_WS"

    elif [ "$#" = "1" ]; then

        case "$user_path" in

            "system"    ) _path="$BENDER_SYSTEM" ;;
            "base"      ) _path="$BENDER_WS/base_ws/src" ;;            
            "soft"      ) _path="$BENDER_WS/soft_ws/src" ;;
            "high"      ) _path="$BENDER_WS/high_ws/src" ;;
            "graveyard" ) _path="$BENDER_WS/bender_code_graveyard" ;;
            "forks"     ) _path="$BENDER_WS/forks_ws/src" ;;
            "embedded"  ) _path="$BENDER_WS/bender_embedded" ;;

             "-h" | "--help" ) show_help=true ;;

            -* | --* )
                echo "Unknown option: $user_path"
                show_help=true ;;

            # unknown --> package
            * )
                pkg_name="$user_path"
                for pkg in $BENDER_PACKAGES
                do
                    if [ "$pkg" = "$pkg_name" ]; then
                        _path=$(rospack find "$pkg_name")
                        cd "$_path"
                        return 0
                    fi
                done


                stack_name="$user_path"
                for stack in $BENDER_STACKS
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
    bender_cd [<workspace>|<package>|-h|--help]

Description:
    It changes the current directory to the root of the
    selected workspace or ROS (meta)package.

Options:
    Available workspace options are:
        - base     : 'cd' to base_ws
        - soft     : 'cd' to soft_ws
        - high     : 'cd' to high_ws
        - system   : 'cd' to bender_system
        - graveyard: 'cd' to bender_code_graveyard
        - forks    : 'cd' to forks_ws
        - embedded : 'cd' to bender_embedded
    
    Available package options correspond to ROS (meta)packages named 'bender_*'.
    
    If no option is given, then the directory will be the one 
    addressed by the \$BENDER_WS environment variable.

EOF
        _bender_admin_goodbye
        return 1
    fi

    cd "$_path"
    return 0
}


# Kill gazebo gently
bender_killgz()
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
    # BENDER_DEPRECATED : mark method as deprecated. The flag is
    # useful for looking up deprecated methods.

    echo "DEPRECATED ... Please call bender_killgz (gently)"
    echo "THIS METHOD WILL BE REMOVED FOR THE NEXT RELEASE"
    bender_killgz
}

bender_net_enable()
{
    python "$BENDER_SYSTEM"/shell/ros_network_indicator/ros_network_indicator.py --enable "$HOME"/bender.sh
    source "$HOME"/bender.sh
}

bender_net_disable()
{
    python "$BENDER_SYSTEM"/shell/ros_network_indicator/ros_network_indicator.py --disable "$HOME"/bender.sh
    source "$HOME"/bender.sh
}

