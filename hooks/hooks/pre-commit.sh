#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# HOOK REQUIREMENTS: 
# see 'pre-commit' file


_bender_git_hooks_revert_stash ()
{
    ## revert changes from stash
    git config apply.whitespace nowarn # prevent stupid warnings
    git reset --hard -q && git stash apply --index -q && git stash drop -q
    git config apply.whitespace ''     # enable the stupid warnings
}

# this function is called when Ctrl-C is sent
# the idea is to revert stashed changes!
_bender_git_hooks_trap_ctrlc ()
{
    # perform cleanup here
    echo "Ctrl-C caught... Reverting Stash"
    _bender_git_hooks_revert_stash
    echo "(OK)"
 
    # exit shell script with error code 2
    # if omitted, shell script will continue execution
    exit 2
}

# last commit
_is_initial_commit="no"
if git rev-parse --verify HEAD >/dev/null 2>&1
then
    against=HEAD
else
    # Initial commit: diff against an empty tree object (magic git hash!)
    against=4b825dc642cb6eb9a060e54bf8d69288fbee4904

    _is_initial_commit="yes"
fi

# Redirect output to stderr.
exec 1>&2



########################################################
# do not accept weird filenames
########################################################

# check nice filenames hook
. "$GITHOOKS_PATH"/pre-commit_filenames.sh


########################################################
# categorization by file extension
########################################################

## current file extension mappings
# - shell : .sh, .bash
# - python: .py
# - xml   : .xml, .urdf, .launch, .xacro, .sdf
# - cpp   : .h, .c, .hpp, .cpp
# - yaml  : .yaml, .yml


# supported formats
ALL_FILES=""
PY_FILES=""
SH_FILES=""
BASH_FILES=""
XML_FILES=""
CPP_FILES=""
YAML_FILES=""
NO_EXT_FILES=""
UNKNOWN_FILES=""


# stash requires initial commit
if [ "$_is_initial_commit" != "yes" ]; then

    #echo "Stashing changes"

    ## modified files
    # stash any changes to the working tree that are not going to be committed
    # then unstash changes to the working tree that we had stashed
    # this way, we only consider changes on the staging area!!
    # .. see: http://stackoverflow.com/questions/20479794/how-do-i-properly-git-stash-pop-in-pre-commit-hooks-to-get-a-clean-working-tree
    old_stash=$(git rev-parse -q --verify refs/stash)
    git stash save -q --keep-index
    new_stash=$(git rev-parse -q --verify refs/stash)

    ## Set trap to ctrl+C (and others), in order to revert the stashed changes
    # initialise trap to call trap_ctrlc function
    # when signal 2 (SIGINT) is received
    trap "_bender_git_hooks_trap_ctrlc" 1 2 15

    # if there were no changes (e.g., `--amend` or `--allow-empty`)
    # then nothing was stashed, and we should skip everything,
    # including the tests themselves.  (Presumably the tests passed
    # on the previous commit, so there is no need to re-run them.)
    if [ "$old_stash" = "$new_stash" ]; then
        #echo "pre-commit script: no changes to test"
        #sleep 1 # XXX hack, editor may erase message
        
        #_bender_git_hooks_revert_stash

        exit 0
    fi
fi


# fullpath to the repo or submodule we are working on.
TOP_LEVEL=$(git rev-parse --show-toplevel)
FILES=$(git diff --cached --name-only --diff-filter=ACMR "$against")
for file in $FILES
do

    _basename=${file##*/}
    #_name=${_basename%.*}
    _ext=${_basename##*.}
    # no extension!
    if [ "$_basename" = "$_ext" ]; then
        _ext=""
    fi

    _fullfile="$TOP_LEVEL/$file"

    ALL_FILES="$ALL_FILES $_fullfile"
    case "$_ext" in

        "py" ) 
            PY_FILES="$PY_FILES $_fullfile" ;;

        "sh" )
            SH_FILES="$SH_FILES $_fullfile" ;;

        "bash" )
            BASH_FILES="$BASH_FILES $_fullfile" ;;

        "xml" | "launch" | "urdf" | "xacro" | "sdf" )
            XML_FILES="$XML_FILES $_fullfile" ;;

        "cpp" | "hpp" | "c" | "h" )
            CPP_FILES="$CPP_FILES $_fullfile" ;;

        "yaml" | "yml" )
            YAML_FILES="$YAML_FILES $_fullfile" ;;

        "" )
            NO_EXT_FILES="$NO_EXT_FILES $_fullfile" ;;

        * )
            UNKNOWN_FILES="$UNKNOWN_FILES $_fullfile" ;;
    esac

    #echo "file: $file"
    #echo "file: $_fullfile"
    #echo "name: ${_name}"
    #echo "ext : ${_ext}"
    #echo ""
done

#echo "PY_FILES: $PY_FILES"
#echo "SH_FILES: $SH_FILES"
#echo "BASH_FILES: $BASH_FILES"
#echo "XML_FILES: $XML_FILES"
#echo "CPP_FILES: $CPP_FILES"
#echo "YAML_FILES: $YAML_FILES"
#echo "NO_EXT_FILES: $NO_EXT_FILES"
#echo "UNKNOWN_FILES: $UNKNOWN_FILES"

########################################################
# run hooks
########################################################

_FAILED="no"

# common stuff
. "$GITHOOKS_PATH"/common.sh

# git merge conflict hook
. "$GITHOOKS_PATH"/pre-commit_merge_conflict.sh

# size hook
. "$GITHOOKS_PATH"/pre-commit_size.sh

# shell hook
. "$GITHOOKS_PATH"/pre-commit_shell.sh

# python hook
. "$GITHOOKS_PATH"/pre-commit_python.sh

# XML hook
. "$GITHOOKS_PATH"/pre-commit_xml.sh

# yaml hook
. "$GITHOOKS_PATH"/pre-commit_yaml.sh

# cpp hook
. "$GITHOOKS_PATH"/pre-commit_cpp.sh


########################################################
# finally ...
########################################################

# stash requires initial commit
if [ "$_is_initial_commit" != "yes" ]; then

    #echo "Reverting changes from stash"
    _bender_git_hooks_revert_stash

fi


## check whether the script succeeded or not
if [ "$_FAILED" != "no" ]; then

   cat <<EOF

 # USE WITH CAUTION!
 to bypass this code inspection you can use:
 > git commit --no-verify -m "..."

 kindly
 git admin.-

 Have some feedback?, please contact us:
 "$BENDER_SYSTEM_ADMIN" - $BENDER_EMAIL_DEVELOP

EOF

    exit 1

fi

exit 0
