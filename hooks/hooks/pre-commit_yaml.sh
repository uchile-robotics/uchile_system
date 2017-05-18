#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - YAML_FILES
# - GITHOOKS_PATH
#
# Exports:
# - _FAILED

# based on PyYaml
# see also: http://pyyaml.org
#  

# check command existence
if ! _uchile_git_hooks_pymodule_exists "yaml"; then
    return 0
fi


_show_yaml_goodbye() {

    cat <<\EOF
 - (FAILED): INVALID YAML FILE(s) ...  BLAME ON YOU!
 Please, fix all yaml errors before commiting.
 Then try again :p

EOF
}


_yaml_failed="no"

_checkfile() {
    
    _file="$1"
    
    python "$GITHOOKS_PATH"/hooks/yaml_linter.py "$_file"

    _rc="$?"
    if [ "$_rc" != "0" ]; then
        #echo "$_rc"
        _yaml_failed="yes"
    fi
}


if [ -z "$YAML_FILES" ]; then
    echo " - (OK) there are no staged yaml files to check :)"
fi

for file in $YAML_FILES
do
    #echo "working on yaml file: "$file
    _checkfile "$file"  
done


# failed
if [ "$_yaml_failed" != "no" ]; then
    _show_yaml_goodbye
    export _FAILED="yes"
else
    if [ -n "$YAML_FILES" ]; then
        echo " - (OK) all staged yaml files have passed the linter."
    fi
fi

