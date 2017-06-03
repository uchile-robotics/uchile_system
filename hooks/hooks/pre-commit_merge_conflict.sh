#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - ALL_FILES
# - GITHOOKS_PATH
#
# Exports:
# - _FAILED


# check python module existence
if ! _uchile_git_hooks_pymodule_exists "argparse"; then
    return 0
fi
if ! _uchile_git_hooks_pymodule_exists "termcolor"; then
    return 0
fi


_show_merge_goodbye() {

    cat <<\EOF
 - (FAILED): FOUND SOME MERGE CONFLICT PATTERNS!...some files contains merge conflict patterns.
 Please, fix them all before commiting. Then try again.

EOF
}


_merge_failed="no"

_checkfile() {
    
    _file="$1"
    python "$GITHOOKS_PATH"/hooks/check_merge_conflict.py "$_file"

    _rc="$?"
    if [ "$_rc" != "0" ]; then
        _merge_failed="yes"
    fi
}


for file in $ALL_FILES
do
    _checkfile "$file"  
done


# failed
if [ "$_merge_failed" != "no" ]; then
    _show_merge_goodbye
    export _FAILED="yes"
else
    echo " - (OK) there are no merge conflict patterns"
fi

