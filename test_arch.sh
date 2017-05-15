#!/bin/sh

cd "$HOME"

UCHILE_WS="$HOME"/uchile_ws
MISC_WS="$UCHILE_WS"/misc
DEPS_WS="$UCHILE_WS"/deps
PKGS_WS="$UCHILE_WS"/pkgs
SYSTEM_WS="$UCHILE_WS"/system
ROS_WS="$UCHILE_WS"/ros

rm -rf "$UCHILE_WS"
mkdir -p "$UCHILE_WS"

mkdir -p "$SYSTEM_WS"

mkdir -p "$MISC_WS"
mkdir -p "$MISC_WS"/wiki
mkdir -p "$MISC_WS"/page
mkdir -p "$MISC_WS"/graveyard
mkdir -p "$MISC_WS"/embedded

mkdir -p "$DEPS_WS"
mkdir -p "$DEPS_WS"/common
mkdir -p "$DEPS_WS"/maqui
mkdir -p "$DEPS_WS"/bender

mkdir -p "$PKGS_WS"

mkdir -p "$PKGS_WS"/forks

mkdir -p "$PKGS_WS"/base # (GIT) [ELIMINAR GIT]
mkdir -p "$PKGS_WS"/base/uchile_common # (GIT PROPUESTO)  algo más a common
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_msgs
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_srvs
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_tf
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_util  # MOVED FROM bender_core_tools/bender_utils 
#mkdir -p "$PKGS_WS"/base/uchile_common/control_util # MOVED FROM bender_hardware/control_util
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_sim   # (GIT PROPUESTO) # duplicacion de mundos! en simuladores... QUe cosas son realmente unicas del robot?
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_sim/uchile_gazebo
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_sim/uchile_stage

mkdir -p "$PKGS_WS"/base/uchile_tools # (GIT PROPUESTO)
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_cmd_vel_mux
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_footprint_generator
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_gui_subtitles
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_laser_pipeline
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_safety
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_turning_base
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_rqt_batteries # que hago con esto?? [RENAME]
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_fun
mkdir -p "$PKGS_WS"/base/uchile_tools/uchile_report_generator

mkdir -p "$PKGS_WS"/base/uchile_knowledge # (GIT)
mkdir -p "$PKGS_WS"/base/uchile_knowledge/uchile_maps
mkdir -p "$PKGS_WS"/base/uchile_knowledge/uchile_db
mkdir -p "$PKGS_WS"/base/uchile_knowledge/uchile_pose_server

mkdir -p "$PKGS_WS"/base/bender_core # (GIT PROPUESTO)
mkdir -p "$PKGS_WS"/base/bender_core/bender_joy # estandar??
mkdir -p "$PKGS_WS"/base/bender_core/bender_description # moved from common
mkdir -p "$PKGS_WS"/base/bender_core/bender_arm
mkdir -p "$PKGS_WS"/base/bender_core/bender_base
mkdir -p "$PKGS_WS"/base/bender_core/bender_fieldbus
mkdir -p "$PKGS_WS"/base/bender_core/bender_gripper
mkdir -p "$PKGS_WS"/base/bender_core/bender_head
mkdir -p "$PKGS_WS"/base/bender_core/bender_sensors
mkdir -p "$PKGS_WS"/base/bender_core/bender_sound
mkdir -p "$PKGS_WS"/base/bender_core/bender_tts
mkdir -p "$PKGS_WS"/base/bender_core/bender_sim 
mkdir -p "$PKGS_WS"/base/bender_core/bender_sim/bender_gazebo
mkdir -p "$PKGS_WS"/base/bender_core/bender_sim/bender_stage

mkdir -p "$PKGS_WS"/base/maqui_core # (GIT PROPUESTO)
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_joy # estandar??
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_description
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_base
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_head
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_sensors # launch normal ros para 2D cam y 3D en pepper!..
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_sound
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_tts
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_sim
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_sim/maqui_gazebo
mkdir -p "$PKGS_WS"/base/maqui_core/maqui_sim/maqui_stage


mkdir -p "$PKGS_WS"/soft # (GIT ELIMINAR [GIT... no funciona!])
mkdir -p "$PKGS_WS"/soft/uchile_hri # (GIT)
mkdir -p "$PKGS_WS"/soft/uchile_hri/uchile_nlp
mkdir -p "$PKGS_WS"/soft/uchile_hri/uchile_speech # INTERFAZ COMUN!
mkdir -p "$PKGS_WS"/soft/uchile_hri/uchile_speech_pocketsphinxs
mkdir -p "$PKGS_WS"/soft/uchile_hri/uchile_speech_nuance
mkdir -p "$PKGS_WS"/soft/uchile_hri/uchile_speech_google

mkdir -p "$PKGS_WS"/soft/uchile_manipulation # (GIT OK)
#mkdir -p "$PKGS_WS"/soft/uchile_manipulation/...

mkdir -p "$PKGS_WS"/soft/uchile_navigation # (GIT OK)
mkdir -p "$PKGS_WS"/soft/uchile_navigation/base_local_planner  # [usar el del fork?]
mkdir -p "$PKGS_WS"/soft/uchile_navigation/uchile_nav
mkdir -p "$PKGS_WS"/soft/uchile_navigation/uchile_navigation_utils
mkdir -p "$PKGS_WS"/soft/uchile_navigation/uchile_twist_recovery

mkdir -p "$PKGS_WS"/soft/uchile_perception # (GIT OK) ... codigo de pato es privado!... consultar con LUZ... sobre repo nuevo o perder historial
#mkdir -p "$PKGS_WS"/soft/uchile_perception/...

#mkdir -p "$PKGS_WS"/soft/uchile_tools # (REMOVE!)

mkdir -p "$PKGS_WS"/soft/maqui_apps # (GIT CREAR?) para cosas pepper roseables desde naoqi, coreographe

mkdir -p "$PKGS_WS"/high # (GIT ELIMINAR)
mkdir -p "$PKGS_WS"/high/uchile_demos   # [NEW REPO] o metapackage
mkdir -p "$PKGS_WS"/high/uchile_robocup # [NEW REPO]
mkdir -p "$PKGS_WS"/high/uchile_skills  # [NEW REPO] MERGE DE SKILLS
mkdir -p "$PKGS_WS"/high/uchile_states  # [NEW REPO]
mkdir -p "$PKGS_WS"/high/bender_bringup # [NEW REPO] merge de bender_core .. machines, launch, configs
mkdir -p "$PKGS_WS"/high/maqui_bringup  # [NEW REPO] merge de bender_core

mkdir -p "$ROS_WS"
mkdir -p "$ROS_WS"/bender_ws
mkdir -p "$ROS_WS"/maqui_ws
mkdir -p "$ROS_WS"/complete_ws
