#!/bin/sh
#
# by mpavez at matias.pavez.b@gmail.com 
#
#
# Required variables:
# - XML_FILES
#
#
# Exports:
# - _FAILED

#
#
# see also:
# - web: http://xmlsoft.org/xmllint.html 
# - $ man xmllint
#  

# check command existence
if ! _uchile_git_hooks_command_exists "xmllint"; then
    return 0
fi

_show_xml_goodbye() {

    cat <<\EOF
 - (FAILED): BLAME ON YOU!
 Please, fix all xml errors before commiting.
 Then try again :p

EOF
}


_xml_failed="no"

_checkfile() {
    
    _file="$1"
    
    # run to help the user
    xmllint --noout "$_file"

    _rc="$?"
    #echo "$_rc"
    if [ "$_rc" != "0" ]; then
        #echo "$_rc"
        _xml_failed="yes"
    fi
}


if [ -z "$XML_FILES" ]; then
    echo " - (OK) there are no staged xml files to check :)"
fi

for file in $XML_FILES
do
    #echo "working on xml file: "$file
    _checkfile "$file"  
done


# failed
if [ "$_xml_failed" != "no" ]; then
    _show_xml_goodbye
    export _FAILED="yes"
else
    if [ -n "$XML_FILES" ]; then
        echo " - (OK) all staged xml files have passed the linter."
    fi
fi

