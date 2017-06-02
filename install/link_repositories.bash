#!/bin/bash
#
# requires the variables:
# - ws_path
# - robot_target

# user input
ws_path="$1"
robot_target="$2"

# checks
if [ -z "$ws_path" ]; then
	printf "Missing first argument ws_path.\n"
	exit 1
fi
if [ -z "$robot_target" ]; then
	printf "Missing second argument robot_target.\n"
	exit 1
fi


# source install tools
source "${ws_path}/system/install/util/helpers.bash"


# load repositories array
declare -a repositories
source "${ws_path}/system/install/robots/${robot_target}.sh"


# create links
n_repositories=${#repositories[@]}
for (( i=1; i<${n_repositories}+1; i++ ));
do
    item="${repositories[$i-1]}"
    item_base="$(echo "$item" | cut -d'/' -f1)"
    item_name="$(echo "$item" | cut -d'/' -f2)"
    
    # create directory if missing
    mkdir -p "${ws_path}/ros/${robot_target}/${item_base}/src/"

    # create symbolic link
    _uchile_link_ "${ws_path}" "${item}" "${robot_target}/${item_base}/src/${item_name}" 
done

unset item
unset n_repositories
unset robot_target
unset ws_path
