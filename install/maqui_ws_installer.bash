#!/bin/bash
#
# installation instructions
# --------------------------
#
# just run this script anywhere (anywhere but the same workspace path!)
#
# $ chmod +x maqui_ws_installer.bash
# $ ./maqui_ws_installer.bash
#

export WS_FOR_ROBOT=maqui


## ======================================================
## WS INSTALLER
## ======================================================

cd $(dirname "$BASH_SOURCE")

# this also sets two env vars:
# . framework_path
# . TMP_SYSTEM_DIR
source ws_installer.bash


## ======================================================
## REPOSITORIES
## ======================================================

printf "\n\n ============ Retrieving Repositories ============ \n"
cd "$framework_path"

# 15 min cache
git config --global credential.helper 'cache --timeout=900'

# where to copy the git hook from
_hook_template="$TMP_SYSTEM_DIR"/hooks/hooks/pre-commit


## MISC
## ----------------------------------------------------------------------------

# wiki

# graveyard


## system
## ----------------------------------------------------------------------------

# system
_uch_get_repository "system" "https://github.com/uchile-robotics/uch_system.git"
_uch_enable_githook "system" "$_hook_template"


## layers
## ----------------------------------------------------------------------------

# base layer

# soft layer

# high layer

# robot_skills


## forks
## ----------------------------------------------------------------------------

# fork: navigation
_uch_get_repository "ros/forks_ws/src/navigation" "https://github.com/uchile-robotics-forks/navigation.git" "kinetic-devel"


## deps
## ----------------------------------------------------------------------------

# install python-aiml
if [ ! -d "$framework_path"/deps/base/knowledge/python-aiml ]; then
	mkdir -p "$framework_path"/deps/base/knowledge/
	cd "$framework_path"/deps/base/knowledge/
	git clone https://github.com/uchile-robotics-forks/python-aiml
	cd python-aiml
	sudo python setup.py install
	cd "$framework_path"
fi

unset _hook_template



# END
# ----------------------------
printf "\n"
printf "The installation is almost ready!. Just follow the installation\n"
printf "instructions on the system README.md file.\n"
printf "\n"

unset framework_path

printf "###########################################\n"
printf " DONE! \n"
printf "###########################################\n"
