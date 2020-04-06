#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - CPP_FILES
#
#
# Exports:
# - _FAILED

#
#
# see also:
# - cppcheck -h
# - http://cppcheck.sourceforge.net
# 

# check command existence
if ! _uchile_git_hooks_command_exists "cppcheck"; then
    return 0
fi


_show_cpp_goodbye() {

    cat <<\EOF
 - (FAILED): BLAME ON YOU!
 Please, fix all cpp errors before commiting.
 Then try again :p

EOF
}


_cpp_failed="no"

_checkfile() {
    
    _file="$1"
    
    cppcheck --error-exitcode=1 --enable=warning --language=c++  --quiet --verbose "$_file"

    _rc="$?"
    #echo "$_rc"
    if [ "$_rc" != "0" ]; then
        #echo "$_rc"
        _cpp_failed="yes"
    fi
}


if [ -z "$CPP_FILES" ]; then
    echo " - (OK) there are no staged cpp files to check :)"
fi

for file in $CPP_FILES
do
    #echo "working on cpp file: "$file
    _checkfile "$file"  
done


# failed
if [ "$_cpp_failed" != "no" ]; then
    _show_cpp_goodbye
    export _FAILED="yes"
else
    if [ -n "$CPP_FILES" ]; then
        echo " - (OK) all staged cpp files have passed the linter."
    fi
fi

