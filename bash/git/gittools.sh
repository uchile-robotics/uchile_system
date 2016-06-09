#!/bin/bash

# TODO:
# fetch en repos nesteados con más de 2 niveles!


BENDER_REPOSITORIES=""
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_SYSTEM"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/base_ws/src"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/base_ws/src/bender_knowledge"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_hri"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_manipulation"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_navigation"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_perception"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_tools"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/high_ws/src"


## 
# _bender_git_status
# - shows a short status of common bender repositiries (see $BENDER_REPOSITORIES)
_bender_git_status ()
{
    local _user_path _repo_path _repo_path_cropped
    _user_path="$(pwd)"

    # echo "git st $*"
    for _repo_path in $BENDER_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$BENDER_WS/}"

        if [ -e "$_repo_path/.git" ]; then
            cd "$_repo_path"
            printf " - - - \n"
            printf "Repository: %s\n" "$_repo_path_cropped"
            git status --short --branch --untracked-files=no
        else
            printf " - - - \n"
            printf "Repository: %s\n" "$_repo_path_cropped"
            printf " - NOT FOUND! \n"
        fi

    done
    cd "$_user_path"

    return 0
}

# permite  cambiar de rama
_bender_git_checkout ()
{
    local _user_path _repo_path _repo_path_cropped _curr_branch _final_branch _modified
    _user_path="$(pwd)"

    _final_branch="$1"
    if [ -z "$_final_branch" ]; then
        printf "Sorry, but you must provide a branch name.\n"
        printf "see: bgit --help\n"
        return 1
    fi

    for _repo_path in $BENDER_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$BENDER_WS/}"

        if [ -e "$_repo_path/.git" ]; then
            cd "$_repo_path"
            _curr_branch="$(git branch --no-color | grep "\*" | sed "s/\* //")"

            printf " - - - \n"
            printf "Repository: %s (%s)\n" "$_repo_path_cropped" "$_curr_branch"
            if [ "$_curr_branch" = "$_final_branch" ]; then
                printf " - already on branch \"%s\".\n" "$_curr_branch"
            else

                _modified="$(git ls-files --modified | wc -l)"
                if [ "$_modified" = "0" ]; then

                    _branch_exists="$(git branch -a --no-color | grep -Ec "[ /]$_final_branch$")"
                    if [ ! "$_branch_exists" = "0" ]; then
                        git checkout "$_final_branch"
                    else
                        printf " - will not checkout: \"%s\" branch does not exists.\n" "$_final_branch"
                    fi
                else
                    printf " - will not checkout: current branch \"%s\" contains %s modified files.\n" "$_curr_branch" "$_modified"
                fi
            fi
        else
            printf " - - - \n"
            printf "Repository: %s\n" "$_repo_path_cropped"
            printf " NOT FOUND! \n"
        fi

    done
    cd "$_user_path"

    return 0
}

_bender_git_ls_files ()
{
    local _user_path _repo_path _repo_path_cropped _command
    _user_path="$(pwd)"

    _command="$1"
    if [ -z "$_command" ]; then
        printf " - Sorry, but you must provide a command name.\n"
        printf " - see: bgit --help\n"
        return 1
    fi
    case "$_command" in
        "modified"  ) _command="ls-files --modified"   ;;
        "untracked" ) _command="ls-files --others"     ;;
        "ignored"   ) _command="check-ignore -v *" ;;
        * )
            printf " - unknown command named \"%s\"\n" "$_command"
            printf " - see: bgit --help\n"
            return 1
            ;;
    esac

    # echo "git st $*"
    for _repo_path in $BENDER_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$BENDER_WS/}"

        if [ -e "$_repo_path/.git" ]; then
            cd "$_repo_path"
            printf " - - - \n"
            printf "Repository: %s\n" "$_repo_path_cropped"
            git $_command
        else
            printf " - - - \n"
            printf "Repository: %s\n" "$_repo_path_cropped"
            printf " - NOT FOUND! \n"
        fi

    done
    cd "$_user_path"

    return 0
}

# trap required to handle some signals and perform the cleanup
# note that the cleanup is performed on _bender_git_fetch not here!  
_bender_git_fetch_trap ()
{
    # this does nothing!
    true
}

_bender_git_fetch ()
{
    local _user_path _repo_path _repo_path_cropped _config
    _user_path="$(pwd)"

    # echo "git st $*"
    for _repo_path in $BENDER_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$BENDER_WS/}"

        # if git repository
        if [ -e "$_repo_path/.git" ]; then

            # skip submodules
            if [ ! -d "$_repo_path/.git" ]; then
                continue
            fi

            cd "$_repo_path"
            printf " - - - \n"
            printf "Fetching repository: %s and submodules ...\n" "$_repo_path_cropped"
            _config="$_repo_path/.git/config"
            _gitmodules="$_repo_path/.gitmodules"

            # deactivate some signals
            # this should prevent leaving the .config, and
            # .gitmodules files in a inconsistent way
            trap "_bender_git_fetch_trap" 1 2 3 15 20
            
            # modify and create config.bkp
            # this replaces any credentials by the default ones: benderuchile:benderrobot on https protocol
            sed --in-place=.bkp "s/https:\/\/\(.*\)bitbucket.org/https:\/\/benderuchile:benderrobot@bitbucket.org/" "$_config"
            if [ -e "$_gitmodules" ]; then
                sed --in-place=.bkp "s/https:\/\/\(.*\)bitbucket.org/https:\/\/benderuchile:benderrobot@bitbucket.org/" "$_gitmodules"
                git submodule --quiet foreach 'sed --in-place=.bkp "s/https:\/\/\(.*\)bitbucket.org/https:\/\/benderuchile:benderrobot@bitbucket.org/" "$toplevel/.git/modules/$name/config"'
            fi
            
            # fetch
            git fetch
            git submodule foreach git fetch

            # restore configs
            # "|| true" is useful to return zero when .bkp does not exists
            mv "$_config".bkp "$_config" 2>/dev/null
            mv "$_gitmodules".bkp "$_gitmodules" 2>/dev/null
            git submodule --quiet foreach 'mv "$toplevel/.git/modules/$name/config.bkp" "$toplevel/.git/modules/$name/config" 2>/dev/null || true'

            # reset signals to defaults
            trap - 1 2 3 15 20
            
        else
            printf " - - - \n"
            printf "Repository: %s\n" "$_repo_path_cropped"
            printf " - NOT FOUND! \n"
        fi
    done
    cd "$_user_path"

    return 0
}


# mismo uso que el comando git, pero para cosas 
# básicas del workspace de bender.
# - bgit status | st
# - bgit checkout
# - bgit ls_files ignored|untracked
bgit () {
 
    local _command _params _show_help
    _command="$1"
    shift 1
    _params="$*"
    
    # echo "command $_command"
    # echo "params  $_params"

    _show_help=
    case "$_command" in
        "st" | "status"    ) _bender_git_status   $_params ;;
        "co" | "checkout"  ) _bender_git_checkout $_params ;;
        "fetch"            ) _bender_git_fetch    $_params ;;
        "ls-files"         ) _bender_git_ls_files $_params ;;
        "-h" | "--help"    ) _show_help=true ;;
        *                  ) _show_help=true ;;
    esac

    if [ "$_show_help" = true ]; then
        cat <<EOF
Synopsis:
    bgit <command> [<options>]

Description:
    It provides a git wrapper for common git needs when working with the bender workspace.

Options:
    Through the 'command' option you can execute a git-like command on every bender workspace.

    Supported values are:
        - status   | st : executes "git status"
        - checkout | co : provides git checkout functionality, only for branch switching
        - ls-files      : looks for untraked or ignored files.
        - --help   | -h : displays this help
        - (empty)       : displays this help

    The lookup is executed on the following workspaces:
    - bender_system
    - base_ws (and submodules)
    - soft_ws (and submodules)
    - high_ws (and submodules)

    excluded repos:
    - forks_ws
    - bender_code_graveyard

EOF
        _bender_admin_goodbye
        return 1
    fi
    return 0
}