#!/bin/bash


## TODO LIST:
# - poder ejecutar el comando sólo en el workspace que se desee
# - poder excluir repos
# - hacer fetch, pull y merge 

BENDER_REPOSITORIES=""
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_SYSTEM"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/base_ws/src"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/base_ws/src/bender_knowledge"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_hri"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_manipulation"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_navigation"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_perception"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/soft_ws/src/bender_tools"
BENDER_REPOSITORIES="$BENDER_REPOSITORIES $BENDER_WS/high_ws/src/"


## 
# _bender_git_status
# - shows a short status of common bender repositiries (see $BENDER_REPOSITORIES)# 
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

                    _branch_exists="$(git branch | grep -c "$_final_branch")"
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

_bender_git_fetch ()
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
            git fetch
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