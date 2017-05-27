#!/bin/sh

# TODO:
# - bgit sólo en workspaces seleccionados
# - fetch en repos nesteados con más de 2 niveles!


UCHILE_REPOSITORIES=""
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_SYSTEM"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/base_ws/uchile_common"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/base_ws/uchile_tools"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/base_ws/uchile_knowledge"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/base_ws/bender_core"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/base_ws/maqui_core"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/soft_ws/uchile_hri"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/soft_ws/uchile_manipulation"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/soft_ws/uchile_navigation"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/soft_ws/uchile_perception"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/high_ws/uchile_high"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/high_ws/bender_bringup"
UCHILE_REPOSITORIES="$UCHILE_REPOSITORIES $UCHILE_PKGS_WS/high_ws/maqui_bringup"


## 
# _uchile_git_status
# - shows a short status of common bender repositiries (see $UCHILE_REPOSITORIES)
_uchile_git_status ()
{
    local _user_path _repo_path _repo_path_cropped
    _user_path="$(pwd)"

	# parse the string array in a bash like manner
    if _uchile_check_if_zsh ; then
        setopt local_options shwordsplit
    fi

    # echo "git st $*"
    for _repo_path in $UCHILE_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$UCHILE_WS\//}"
        _repo_path_cropped="${_repo_path_cropped//pkgs\//}"

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
_uchile_git_checkout ()
{
    local _user_path _repo_path _repo_path_cropped _curr_branch _final_branch _modified
    _user_path="$(pwd)"

    _final_branch="$1"
    if [ -z "$_final_branch" ]; then
        printf "Sorry, but you must provide a branch name.\n"
        printf "see: bgit --help\n"
        return 1
    fi

    # parse the string array in a bash like manner
    if _uchile_check_if_zsh ; then
        setopt local_options shwordsplit
    fi

    for _repo_path in $UCHILE_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$UCHILE_WS\//}"
        _repo_path_cropped="${_repo_path_cropped//pkgs\//}"

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

_uchile_git_ls_files ()
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

	# parse the string array in a bash like manner
    if _uchile_check_if_zsh ; then
        setopt local_options shwordsplit
    fi

    # echo "git st $*"
    for _repo_path in $UCHILE_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$UCHILE_WS\//}"
        _repo_path_cropped="${_repo_path_cropped//pkgs\//}"

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

_uchile_git_fetch ()
{
    local _user_path _repo_path _repo_path_cropped
    _user_path="$(pwd)"

    # parse the string array in a bash like manner
    if _uchile_check_if_zsh ; then
        setopt local_options shwordsplit
    fi

    # echo "git st $*"
    for _repo_path in $UCHILE_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$UCHILE_WS\//}"
        _repo_path_cropped="${_repo_path_cropped//pkgs\//}"

        # if git repository
        if [ -e "$_repo_path/.git" ]; then

            # skip submodules
            if [ ! -d "$_repo_path/.git" ]; then
                continue
            fi

            cd "$_repo_path"
            printf " - - - \n"
            printf "Fetching repository: %s and submodules ...\n" "$_repo_path_cropped"
            git fetch
            git submodule foreach git fetch
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
# note that cleanup depends and is performed on the caller, not here!.
_uchile_git_trap ()
{
    # this does nothing!
    true
}

_uchile_git_merge_common ()
{
    local _curr_remote _curr_branch
    _curr_branch="$(git rev-parse --abbrev-ref HEAD)"
    _curr_remote="origin/$_curr_branch"

    # remote does not exitsts
    _branch_exists="$(git branch -a --no-color | grep -Ec "[ /]$_curr_remote")"
    if [ "$_branch_exists" = "0" ]; then
        printf " - will not merge %s does not exits.\n" "$_curr_remote"
        return 1
    fi

    # (remote <= branch) ==> ALREADY UP TO DATE
    if git merge-base --is-ancestor "$_curr_remote" "$_curr_branch"; then
        printf " - already up-to-date\n"
    else
        # (branch <= remote) ==> SHOULD UPDATE
        if git merge-base --is-ancestor "$_curr_branch" "$_curr_remote"; then
            printf " - merge: (%s) <---- (%s)\n" "$_curr_branch" "$_curr_remote"
            git merge "$_curr_remote"
        else
            # NON FAST FORDWARD
            printf " - will not merge %s onto %s (not fast-forward merge)\n" "$_curr_remote" "$_curr_branch"
        fi
    fi
}

_uchile_git_merge ()
{
    local _user_path _repo_path _repo_path_cropped
    _user_path="$(pwd)"

    # parse the string array in a bash like manner
    if _uchile_check_if_zsh ; then
        setopt local_options shwordsplit
    fi

    # echo "git st $*"
    for _repo_path in $UCHILE_REPOSITORIES; do

        # short version
        _repo_path_cropped="${_repo_path//$UCHILE_WS\//}"
        _repo_path_cropped="${_repo_path_cropped//pkgs\//}"

        # if git repository
        if [ -e "$_repo_path/.git" ]; then

            # skip submodules
            if [ ! -d "$_repo_path/.git" ]; then
                continue
            fi

            cd "$_repo_path"
            printf " - - - \n"
            printf "repository: %s and submodules ...\n" "$_repo_path_cropped"

            # deactivate some signals
            # IS THIS REALLY NEEDED? 
            # ... maybe not. but just to be sure nobody breaks the merge
            # ''\_(*u*)_/''
            if _uchile_check_if_bash ; then
                trap "_uchile_git_trap" 1 2 3 15 20
            else
                # NOT IMPLEMENTED TRAP
                : # noop
            fi

            # merge
            _uchile_git_merge_common
            git submodule foreach _uchile_git_merge_common

            # reset signals to defaults
            if _uchile_check_if_bash ; then
                trap - 1 2 3 15 20
            else
                # NOT IMPLEMENTED TRAP EXIT
                : # noop
            fi
            
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
# TODO:
# |   $ bgit merge <branchname>
# |   $ bgit merge origin/develop
# |   $ bgit merge develop
bgit ()
{
 
    local _command _params _show_help
    _command="$1"
    shift 1
    _params="$*"
    
    # echo "command $_command"
    # echo "params  $_params"

    _show_help=
    case "$_command" in
        "st" | "status"    ) _uchile_git_status            ;;
        "co" | "checkout"  ) _uchile_git_checkout $_params ;;
        "fetch"            ) _uchile_git_fetch             ;;
        "merge"            ) _uchile_git_merge             ;;
        "pull"             ) _uchile_git_fetch
                             _uchile_git_merge             ;;
        "ls-files"         ) _uchile_git_ls_files $_params ;;
        "-h" | "--help"    ) _show_help=true ;;
        *                  ) _show_help=true ;;
    esac

    if [ "$_show_help" = true ]; then
        cat <<EOF
Synopsis:
    bgit <command> [<options>]

Description:
    It provides a git wrapper for common git needs when working with the bender
    workspace.

    Every command will be executed on all bender repositories, one by one.
    This way, a single command (bgit) provides you a way to see the workspace
    status (bgit st) or to request a fetch from all the remotes (bgit fetch).


Options:
    Through the 'command' option you can execute a git-like command on every 
    bender workspace.

    Supported values are:
        - status   | st : Executes "git status"

        - checkout | co : Provides git checkout functionality, only for branch
                          switching.

                          The checkout will only proceed for a repo, if it is
                          in a clean state, with no modified files.

                          Autocomplete feature is only available for master and
                          develop branches.

                          usage:
                          |   $ bgit checkout <branchname>
                          |   $ bgit checkout develop
                          |   $ bgit checkout feat-foo


        - merge         : Provides git merge functionality, only from remotes
                          like origin/<current_branchname>. 

                          Only fast-forward merges are executed!, otherwise the
                          merge will not proceed.

                          usage:
                          |   $ bgit merge


        - pull          : Alias for:
                          |   $ bgit fetch
                          |   $ bgit merge


        - ls-files      : It looks for modified, untraked or ignored files.

                          usage:
                          |   $ bgit ls-files modified
                          |   $ bgit ls-files untraked
                          |   $ bgit ls-files ignored


        - --help   | -h : Displays this help

        - (empty)       : Displays this help

    The lookup is executed on the following workspaces:
    - system
    - base_ws (and submodules)
    - soft_ws (and submodules)
    - high_ws (and submodules)

    excluded repos:
    - forks_ws
    - graveyard

EOF
        _uchile_admin_goodbye
        return 1
    fi
    return 0
}