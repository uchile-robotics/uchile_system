#!/bin/sh

##############################################################################################
#   OVERVIEW
##############################################################################################
# bender_find_string        - finds string within bender src space
# bender_cd                 - cd to a bender framework directory
# cdb                       - the same as bender_cd but faster to type 
# bender_ws                 - opens a window on a bender framework directory 
# bender_printenv           - prints all BENDER_* environment variables
# bender_refresh_bash       - executes bash to resource the bender framework
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

    ## bender_refresh_bash
    # executes bash to resource the bender framework.
    #
    # this is for testing purposes only!. Do not use it
    # when environment variables have changed. Open a new
    # terminal session instead.
    function bender_refresh_bash
    {
        exec bash
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
    function cdb
    {
        bender_cd
    }
    
fi




bender_open_config () {

    local _conf _editor
    _conf="$HOME"/bender.sh

    # check file existence
    if [ ! -e "$_conf" ]; then
        printf "User configuration file not found: %s\n" "$_conf"
        return 1
    fi

    # check if EDITOR variable is unset
    _editor="$EDITOR"
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
    local string path user_path opt show_help curpath

    path=""
    path="$path $BENDER_SYSTEM"         # bender_system
    path="$path $BENDER_WS/base_ws/src" # base_ws
    path="$path $BENDER_WS/soft_ws/src" # soft_ws
    path="$path $BENDER_WS/high_ws/src" # high_ws

    string=""
    if [ "$#" = "2" ]; then 

        opt="$1"
        case "$opt" in

            "system"    ) path="$BENDER_SYSTEM" ;;
            "base"      ) path="$BENDER_WS/base_ws/src" ;;
            "soft"      ) path="$BENDER_WS/soft_ws/src" ;;
            "high"      ) path="$BENDER_WS/high_ws/src" ;;
            "graveyard" ) path="$BENDER_WS/bender_code_graveyard" ;;
            "forks"     ) path="$BENDER_WS/forks_ws/src" ;;
            "embedded"  ) path="$BENDER_WS/bender_embedded" ;;
            "all" )
                path="$path $BENDER_WS/bender_embedded"
                path="$path $BENDER_WS/forks_ws/src"
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

    for curpath in $path; do

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
bender_cd ()
{
    local user_path show_help path pkg_name pkg stack_name stack
    path=""

    user_path="$1"

    if [ "$#" = "0" ]; then
        path="$BENDER_WS"

    elif [ "$#" = "1" ]; then

        case "$user_path" in

            "system"    ) path="$BENDER_SYSTEM" ;;
            "base"      ) path="$BENDER_WS/base_ws/src" ;;            
            "soft"      ) path="$BENDER_WS/soft_ws/src" ;;
            "high"      ) path="$BENDER_WS/high_ws/src" ;;
            "graveyard" ) path="$BENDER_WS/bender_code_graveyard" ;;
            "forks"     ) path="$BENDER_WS/forks_ws/src" ;;
            "embedded"  ) path="$BENDER_WS/bender_embedded" ;;

             "-h" | "--help" ) show_help=true ;;

            -* | --* )
                echo "Unknown option: $user_path"
                show_help=true ;;

            # unknown --> package
            * )
                pkg_name="$user_path"
                for pkg in $BENDER_PACKAGES; do
                    if [ "$pkg" = "$pkg_name" ]; then
                        path=$(rospack find "$pkg_name")
                        cd "$path"
                        return 0
                    fi
                done

                stack_name="$user_path"
                for stack in $BENDER_STACKS; do
                    if [ "$stack" = "$stack_name" ]; then
                        path=$(rosstack find "$stack_name")
                        cd "$path"/..
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

    cd "$path"
    return 0
}

## bender_ws
# see also: bender_ws --help
bender_ws ()
{
    local user_path show_help path pkg pkg_name stack stack_name
    path=""

    user_path="$1"

    if [ "$#" = "0" ]; then
        path="$BENDER_WS"

    elif [ "$#" = "1" ]; then

        case "$user_path" in

            "system"    ) path="$BENDER_SYSTEM" ;;
            "base"      ) path="$BENDER_WS/base_ws/src" ;;            
            "soft"      ) path="$BENDER_WS/soft_ws/src" ;;
            "high"      ) path="$BENDER_WS/high_ws/src" ;;
            "graveyard" ) path="$BENDER_WS/bender_code_graveyard" ;;
            "forks"     ) path="$BENDER_WS/forks_ws/src" ;;
            "embedded"  ) path="$BENDER_WS/bender_embedded" ;;

             "-h" | "--help" ) show_help=true ;;

            -* | --* )
                echo "Unknown option: $user_path"
                show_help=true ;;

            # unknown --> package
            * )
                pkg_name="$user_path"
                for pkg in $BENDER_PACKAGES; do
                    if [ "$pkg" = "$pkg_name" ]; then
                        path=$(rospack find "$pkg_name")
                        nautilus "$path"
                        return 0
                    fi
                done

                stack_name="$user_path"
                for stack in $BENDER_STACKS; do
                    if [ "$stack" = "$stack_name" ]; then
                        path=$(rosstack find "$stack_name")
                        nautilus "$path"
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
    bender_ws [<workspace>|<package>|-h|--help]

Description:
    It opens a nautilus window on the the selected workspace or ROS (meta)package.

Options:
    Available workspace options are:
        - base     : 'nautilus' on base_ws
        - soft     : 'nautilus' on soft_ws
        - high     : 'nautilus' on high_ws
        - system   : 'nautilus' on bender_system
        - graveyard: 'nautilus' on bender_code_graveyard
        - forks    : 'nautilus' on forks_ws
        - embedded : 'nautilus' on bender_embedded
    
    Available package options correspond to ROS (meta)packages named 'bender_*'.
    
    If no option is given, then the directory will be the one 
    addressed by the \$BENDER_WS environment variable.

EOF
        _bender_admin_goodbye
        return 1
    fi

    nautilus "$path"
    return 0
}

# Kill gazebo gently
killgz(){
    # Kill controllers spawners
    rosnode kill /bender/controller_spawner 
    rosnode kill /bender/neck_controller_spawner
    # Kill Gazebo server
    killall gzserver
    # Kill Gazebo client
    killall gzclient
}
