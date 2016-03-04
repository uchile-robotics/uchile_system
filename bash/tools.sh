#!/bin/sh

##############################################################################################
#   OVERVIEW
##############################################################################################
# bender_find_string  -   finds string within bender src space
# bender_cd           -   



##############################################################################################
#   shell utilities
##############################################################################################

alias bender_ws='nautilus $BENDER_WS'
alias bender_printenv='printenv | grep "BENDER_.*="'
alias bender_refresh_bash='exec bash'
alias bender_git_show_untracked='git ls-files --others'
alias bender_git_show_ignored='git check-ignore -v *'
alias cdb='bender_cd'

## bender_find_string
# see also: bender_find_string --help 
bender_find_string ()
{
    # TODO: consider filenames

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
            "install"   ) path="$BENDER_WS/install" ;;
            "embedded"  ) path="$BENDER_WS/bender_embedded" ;;
            "all" )
                path="$path $BENDER_WS/bender_code_graveyard"
                path="$path $BENDER_WS/bender_embedded"
                path="$path $BENDER_WS/forks_ws/src"
                path="$path $BENDER_WS/install"
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
        - install  : lookup on the install folder at <bender_ws>/install
        - embedded : lookup on bender_embedded
        - all      : lookup on all previous locations

    By default the lookup is executed on system-base-soft-high.
EOF
        return 1
    fi
    
    # i used this "cd" trick because we don't want fullpaths
    # displayed by grep, but shorter ones
    user_path=$(pwd)

    for curpath in $path; do
        echo "$curpath"

        echo "[INFO]: Looking for '$string' on path: $curpath"
        cd "$curpath"
        grep -rInH --exclude-dir="\.git" "$string" .

    done

    cd "$user_path"

    return 0
}

## bender_cd
# see also: bender_cd --help
bender_cd ()
{
    local user_path show_help path
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
            "install"   ) path="$BENDER_WS/install" ;;
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
                echo "Bender package named '$pkg_name' doesn't not exists. Try with 'roscd' command."
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
    selected workspace or ROS package.

Options:
    Available workspace options are:
        - base     : 'cd' to base_ws
        - soft     : 'cd' to soft_ws
        - high     : 'cd' to high_ws
        - system   : 'cd' to bender_system
        - graveyard: 'cd' to bender_code_graveyard
        - forks    : 'cd' to forks_ws
        - install  : 'cd' to the install folder at <bender_ws>/install
        - embedded : 'cd' to bender_embedded
    
    Available package options correspond to ROS packages named 'bender_*'.
    
    If no option is given, then the directory will be the one 
    addressed by the \$BENDER_WS environemnt variable.

EOF
    fi

    cd "$path"
    return 0
}
