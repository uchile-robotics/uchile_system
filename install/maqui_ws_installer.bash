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


# ## MISC
# ## ----------------------------------------------------------------------------

# # wiki
# _uch_get_repository "misc/wiki" "https://bitbucket.org/uchile-robotics-die/bender_system.git/wiki"

# # code_graveyard
# # OBS: no queremos hooks en el graveyard!, es codigo que no vale la pena arreglar
# _uch_get_repository "misc/code_graveyard" "https://bitbucket.org/uchile-robotics-die/bender_code_graveyard.git"

# # bender_embedded
# _uch_get_repository "misc/embedded" "https://bitbucket.org/uchile-robotics-die/bender_embedded.git"


## system
## ----------------------------------------------------------------------------

# system
_uch_get_repository "system" "https://github.com/uchile-robotics/uch_system.git"
_uch_enable_githook "system" "$_hook_template"


## layers
## ----------------------------------------------------------------------------

# # base layer
# _uch_get_repository_for_ws "ros/base_ws/src" "https://bitbucket.org/uchile-robotics-die/bender_base_layer.git"
# _uch_enable_githook "ros/base_ws/src" "$_hook_template"

# # soft layer
# _uch_get_repository_for_ws "ros/soft_ws/src" "https://bitbucket.org/uchile-robotics-die/bender_soft_layer.git"
# _uch_enable_githook "ros/soft_ws/src" "$_hook_template"

# # high layer
# _uch_get_repository_for_ws "ros/high_ws/src" "https://bitbucket.org/uchile-robotics-die/bender_high_layer.git"
# _uch_enable_githook "ros/high_ws/src" "$_hook_template"

# # robot_skills
# _uch_get_repository "ros/high_ws/src/robot_skills" "https://github.com/uchile-robotics/robot_skills.git"
# _uch_enable_githook "ros/high_ws/src/robot_skills" "$_hook_template"


# ## forks
# ## ----------------------------------------------------------------------------

# # forks: rosaria
# _uch_get_repository "ros/forks_ws/src/rosaria" "https://github.com/uchile-robotics-forks/rosaria.git" "master"

# # fork: dynamixel_motor
# _uch_get_repository "ros/forks_ws/src/dynamixel_motor" "https://github.com/uchile-robotics-forks/dynamixel_motor.git" "develop"

# # fork: usb_cam
# _uch_get_repository "ros/forks_ws/src/usb_cam" "https://github.com/uchile-robotics-forks/usb_cam.git" "0.3.4"

# # fork: urg_node
# _uch_get_repository "ros/forks_ws/src/urg_node" "https://github.com/uchile-robotics-forks/urg_node.git" "0.1.9"

# # fork: navigation
# _uch_get_repository "ros/forks_ws/src/navigation" "https://github.com/uchile-robotics-forks/navigation.git" "kinetic-devel"


# ## deps
# ## ----------------------------------------------------------------------------

# # install python-aiml
# if [ ! -d "$framework_path"/deps/base/knowledge/python-aiml ]; then
# 	mkdir -p "$framework_path"/deps/base/knowledge/
# 	cd "$framework_path"/deps/base/knowledge/
# 	git clone https://github.com/uchile-robotics-forks/python-aiml
# 	cd python-aiml
# 	sudo python setup.py install
# 	cd "$framework_path"
# fi

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
