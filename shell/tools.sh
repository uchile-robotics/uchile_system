#!/bin/sh

###############################################################################
#   OVERVIEW
###############################################################################
# uchile_find_string        - finds string within uchile src space
# uchile_cd                 - cd to a robot framework directory
# cdb, cdu                  - the same as uchile_cd but faster to type. The "b"
#                             stands for Bender and legacy users.
# uchile_printenv           - prints all UCHILE_* environment variables
# uchile_refresh_shell      - reexecutes a shell to resource the robot framework
# uchile_git_show_untracked - lists currently untracked files
# uchile_git_show_ignored   - lists currently ignored files

###############################################################################
#   shell utilities
###############################################################################

# prevent failure
if _uchile_check_if_bash_or_zsh ; then

    # prints all UCHILE_* environment variables and its values
    uchile_printenv ()
    {
        printenv | sort | grep "UCHILE_.*=" 
    }

    ## uchile_refresh_shell
    # executes bash to resource the robot framework.
    #
    # this is for testing purposes only!. Do not use it
    # when environment variables have changed. Open a new
    # terminal session instead.
    uchile_refresh_shell ()
    {
        export UCHILE_FRAMEWORK_TWICE_LOAD_CHECK=false
        exec "$UCHILE_FRAMEWORK_LOADED_SHELL"
    }
    

    # lists currently untracked files
    uchile_git_show_untracked ()
    {
        git ls-files --others
    }

    # lists currently ignored files
    uchile_git_show_ignored ()
    {
        git check-ignore -v *
    }

    # the same as uchile_cd but faster to type 
    alias cdb="uchile_cd"
    alias cdu="uchile_cd"
    alias bviz="roslaunch uchile_util rviz.launch"
    
fi


uchile_open_config ()
{
    local _conf _editor
    _conf="$UCHILE_SHELL_CFG"

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
    if ! _uchile_check_var_isset "EDITOR"; then
        printf "EDITOR env variable is unset. Using 'cat'\n"
        _editor="cat"
    fi
    printf " - EDITOR env variable resolves to: '%s'\n" "$_editor"

    # check editor is installed
    if ! _uchile_check_installed "$_editor"; then
        printf "Sorry, but '%s' is not installed. Using 'cat'\n" "$_editor"
        _editor="cat"
    fi


    printf " - opening user config file '%s' with '%s'\n" "$_conf" "$_editor"    
    "$_editor" "$_conf" &
}

## uchile_find_string
# see also: uchile_find_string --help 
uchile_find_string ()
{
    local string _path user_path opt show_help curpath

    _path="$UCHILE_SYSTEM"                 # system
    _path="$_path $UCHILE_PKGS_WS/base_ws" # base_ws
    _path="$_path $UCHILE_PKGS_WS/soft_ws" # soft_ws
    _path="$_path $UCHILE_PKGS_WS/high_ws" # high_ws

    string=""
    if [ "$#" = "2" ]; then 

        opt="$1"
        case "$opt" in

            "system"    ) _path="$UCHILE_SYSTEM" ;;
            "forks"     ) _path="$UCHILE_PKGS_WS/forks_ws" ;;
            "base"      ) _path="$UCHILE_PKGS_WS/base_ws" ;;
            "soft"      ) _path="$UCHILE_PKGS_WS/soft_ws" ;;
            "high"      ) _path="$UCHILE_PKGS_WS/high_ws" ;;
            "misc"      ) _path="$UCHILE_MISC_WS" ;;
            "deps"      ) _path="$UCHILE_DEPS_WS" ;;
            "pkgs"      ) _path="$UCHILE_PKGS_WS" ;;
            "all" )
                _path="$_path $UCHILE_PKGS_WS/forks_ws"
                _path="$_path $UCHILE_DEPS_WS"
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
    uchile_find_string [<workspace>] <string>

Description:
    It looks for instances of a <string> written on any file
    located on at least 1 robot workspace.

Options:
    Through the 'workspace' option you can change the location
    where lookup will be executed.

    Supported values are:
        - forks  : lookup on forks_ws
        - base   : lookup on base_ws
        - soft   : lookup on soft_ws
        - high   : lookup on high_ws
        - system : lookup on system
        - deps   : lookup on deps ws
        - all    : lookup on all previous locations
        - pkgs   : lookup on pkgs ws
        - misc   : lookup on misc ws

    By default the lookup is executed on system-base-soft-high.
EOF

        _uchile_admin_goodbye
        return 1
    fi
    
    # i used this "cd" trick because we don't want fullpaths
    # displayed by grep, but shorter ones
    user_path=$(pwd)

    # parse the string array in a bash like manner
    if _uchile_check_if_zsh ; then
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
        find . -wholename "*$string*" -print -o -path "*/.git" -prune | grep "$string"
    done

    cd "$user_path"

    return 0
}

## uchile_cd
# see also: uchile_cd --help
uchile_cd ()
{
    local user_path show_help _path pkg_name pkg stack_name stack
    _path=""

    if _uchile_check_if_zsh ; then
        setopt local_options shwordsplit
    fi
    
    user_path="$1"

    if [ "$#" = "0" ]; then
        _path="$UCHILE_ROS_WS"

    elif [ "$#" = "1" ]; then

        case "$user_path" in

            "system" ) _path="$UCHILE_SYSTEM" ;;
            "forks"  ) _path="$UCHILE_ROS_WS/forks_ws/src" ;;
            "base"   ) _path="$UCHILE_ROS_WS/base_ws/src" ;;            
            "soft"   ) _path="$UCHILE_ROS_WS/soft_ws/src" ;;
            "high"   ) _path="$UCHILE_ROS_WS/high_ws/src" ;;
            "misc"   ) _path="$UCHILE_MISC_WS" ;;
            "deps"   ) _path="$UCHILE_DEPS_WS" ;;
            "pkgs"   ) _path="$UCHILE_PKGS_WS" ;;

             "-h" | "--help" ) show_help=true ;;

            -* | --* )
                echo "Unknown option: $user_path"
                show_help=true ;;

            # unknown --> package
            * )
                pkg_name="$user_path"
                for pkg in $UCHILE_PACKAGES
                do
                    if [ "$pkg" = "$pkg_name" ]; then
                        _path=$(rospack find "$pkg_name")
                        cd "$_path"
                        return 0
                    fi
                done


                stack_name="$user_path"
                for stack in $UCHILE_STACKS
                do
                    if [ "$stack" = "$stack_name" ]; then
                        _path=$(rosstack find "$stack_name")
                        cd "$_path"/..
                        return 0
                    fi
                done

                echo "(meta)package named '$pkg_name' doesn't not found on the UChile ROS Framework. Try with 'roscd' command."
                show_help=true
        esac
    else
        show_help=true
    fi

    if [ "$show_help" = true ]; then
        cat <<EOF
Synopsis:                
    uchile_cd [<workspace>|<package>|-h|--help]

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
        - misc     : 'cd' to misc workspace
        - deps     : 'cd' to deps workspace
        - pkgs     : 'cd' to pkgs workspace
    
    Available package options correspond to ROS (meta)packages found on the UChile ROS Framework.
    
    If no option is given, then the directory will be the one 
    addressed by the \$UCHILE_ROS_WS environment variable.

EOF
        _uchile_admin_goodbye
        return 1
    fi

    cd "$_path"
    return 0
}


# Kill gazebo gently
uchile_killgz ()
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
    # UCHILE_DEPRECATED : mark method as deprecated. The flag is
    # useful for looking up deprecated methods.

    echo "DEPRECATED ... Please call uchile_killgz (gently)"
    echo "THIS METHOD WILL BE REMOVED FOR THE NEXT RELEASE"
    uchile_killgz
}

uchile_net_enable ()
{
    python "$UCHILE_SYSTEM"/shell/ros_network_indicator/ros_network_indicator.py --enable "$HOME"/uchile.sh
    . "$HOME"/uchile.sh
}

uchile_net_disable ()
{
    python "$UCHILE_SYSTEM"/shell/ros_network_indicator/ros_network_indicator.py --disable "$HOME"/uchile.sh
    . "$HOME"/uchile.sh
}


uchile_clean_workspace ()
{
    # ask and clean !
    if [ -d "${UCHILE_ROS_WS}" ]; then
        if [ ! -z "$(ls -A ${UCHILE_ROS_WS})" ]; then

            printf  " - Directory %s exists and is not empty!\n" "${UCHILE_ROS_WS}"
            if _uchile_check_if_zsh ; then
                read -q "REPLY?   Do you want to overwrite it? [Y/n]"
                echo
            else
                read -p "   Do you want to overwrite it? [Y/n]" -r
            fi
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                printf " - Deleting workspace overlay.\n"
                rm -rf "${UCHILE_ROS_WS}"
            else
                printf " - Understood. Bye!.\n"
                return 0
            fi
        fi
    fi
    mkdir -p "${UCHILE_ROS_WS}"

    # remake workspaces
    bash -c "source ${UCHILE_SYSTEM}/install/util/helpers.bash; _uchile_create_complete_ws ${UCHILE_ROS_WS}"

    # link missing repositories
    uchile_fix_links

    return 0
}


uchile_fix_links ()
{
    # link missing repositories
    printf  " - Creating missing links for %s repositories.\n" "${UCHILE_ROBOT}"
    bash "${UCHILE_SYSTEM}"/install/link_repositories.bash "${UCHILE_WS}" "${UCHILE_ROBOT}"

    return 0
}