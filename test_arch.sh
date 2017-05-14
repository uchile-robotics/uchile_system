#!/bin/sh

cd "$HOME"

UCHILE_WS="$HOME"/uch_ws
MISC_WS="$UCHILE_WS"/misc
DEPS_WS="$UCHILE_WS"/deps
PKGS_WS="$UCHILE_WS"/pkgs
SYSTEM_WS="$UCHILE_WS"/system
ROS_WS="$UCHILE_WS"/ros

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
mkdir -p "$PKGS_WS"/base # (GIT)
mkdir -p "$PKGS_WS"/base/uchile_common # (GIT PROPUESTO)  algo más a common
mkdir -p "$PKGS_WS"/base/uchile_common/bender_description # (GIT PROPUESTO) # consultar rorro,... que debiera tener esto comunmente?
mkdir -p "$PKGS_WS"/base/uchile_common/maqui_description  # (GIT PROPUESTO) # consultar rorro,... que debiera tener esto comunmente?
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_msgs
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_srvs
mkdir -p "$PKGS_WS"/base/uchile_common/uchile_tf
mkdir -p "$PKGS_WS"/base/uchile_core_tools # (GIT PROPUESTO)
mkdir -p "$PKGS_WS"/base/uchile_core_tools/uchile_cmd_vel_mux
mkdir -p "$PKGS_WS"/base/uchile_core_tools/uchile_footprint_generator
mkdir -p "$PKGS_WS"/base/uchile_core_tools/uchile_gui_subtitles
mkdir -p "$PKGS_WS"/base/uchile_core_tools/uchile_laser_pipeline
mkdir -p "$PKGS_WS"/base/uchile_core_tools/uchile_safety
mkdir -p "$PKGS_WS"/base/uchile_core_tools/uchile_turning_base
mkdir -p "$PKGS_WS"/base/uchile_core_tools/uchile_utils  # .. common?
mkdir -p "$PKGS_WS"/base/uchile_core_tools/rqt_batteries # que hago con esto?? 
mkdir -p "$PKGS_WS"/base/bender_hardware # (GIT PROPUESTO)
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_arm
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_base
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_fieldbus
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_gripper
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_head
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_joy # estandar
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_sensors # launch normal ros para 2D cam y 3D en pepper!..
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_sound
mkdir -p "$PKGS_WS"/base/bender_hardware/bender_tts
mkdir -p "$PKGS_WS"/base/bender_hardware/control_util # que hacer con esto? -- common?
mkdir -p "$PKGS_WS"/base/maqui_hardware # (GIT PROPUESTO)
mkdir -p "$PKGS_WS"/base/uchile_knowledge # (GIT)
mkdir -p "$PKGS_WS"/base/uchile_knowledge/uchile_maps
mkdir -p "$PKGS_WS"/base/uchile_knowledge/uchile_db
mkdir -p "$PKGS_WS"/base/uchile_knowledge/uchile_pose_server
mkdir -p "$PKGS_WS"/base/uchile_sim # (GIT PROPUESTO)
mkdir -p "$PKGS_WS"/base/uchile_sim/uchile_gazebo
mkdir -p "$PKGS_WS"/base/uchile_sim/uchile_stage
mkdir -p "$PKGS_WS"/base/uchile_sim/bender_gazebo
mkdir -p "$PKGS_WS"/base/uchile_sim/maqui_gazebo
mkdir -p "$PKGS_WS"/base/uchile_sim/bender_stage
mkdir -p "$PKGS_WS"/base/uchile_sim/maqui_stage
mkdir -p "$PKGS_WS"/base/bender_sim # duplicacion de mundos! en simuladores... QUe cosas son realmente unicas del robot?
mkdir -p "$PKGS_WS"/base/bender_sim/bender_gazebo # robot + launch hooks/versions + controllers? 
mkdir -p "$PKGS_WS"/base/bender_sim/bender_mock   # NADA!,, REMOVER!
mkdir -p "$PKGS_WS"/base/bender_sim/bender_stage  # desc. robot + hooks
mkdir -p "$PKGS_WS"/base/maqui_sim
mkdir -p "$PKGS_WS"/base/maqui_sim/maqui_gazebo
mkdir -p "$PKGS_WS"/base/maqui_sim/maqui_mock
mkdir -p "$PKGS_WS"/base/maqui_sim/maqui_stage
mkdir -p "$PKGS_WS"/soft # (GIT)
mkdir -p "$PKGS_WS"/soft/uchile_hri # (GIT)
mkdir -p "$PKGS_WS"/soft/uchile_hri/uchile_nlp
mkdir -p "$PKGS_WS"/soft/uchile_hri/uchile_speech
mkdir -p "$PKGS_WS"/soft/bender_manipulation # (GIT)
mkdir -p "$PKGS_WS"/soft/uchile_navigation # (GIT)
mkdir -p "$PKGS_WS"/soft/uchile_navigation/base_local_planner
mkdir -p "$PKGS_WS"/soft/uchile_navigation/uchile_nav
mkdir -p "$PKGS_WS"/soft/uchile_navigation/uchile_navigation_utils
mkdir -p "$PKGS_WS"/soft/uchile_navigation/uchile_twist_recovery
mkdir -p "$PKGS_WS"/soft/uchile_perception # (GIT)  algunas cosas son pepper roseables
mkdir -p "$PKGS_WS"/soft/uchile_perception/...
mkdir -p "$PKGS_WS"/soft/uchile_tools # (GIT)
mkdir -p "$PKGS_WS"/soft/uchile_tools/uchile_fun
mkdir -p "$PKGS_WS"/soft/uchile_tools/uchile_report_generator
mkdir -p "$PKGS_WS"/high # (GIT) OK!
mkdir -p "$PKGS_WS"/high/uchile_demos
mkdir -p "$PKGS_WS"/high/uchile_robocup
mkdir -p "$PKGS_WS"/high/uchile_bringup # merge de bender_core
mkdir -p "$PKGS_WS"/high/uchile_skills # MERGE DE SKILLS
mkdir -p "$PKGS_WS"/high/uchile_states
mkdir -p "$ROS_WS"
mkdir -p "$ROS_WS"/bender_ws
mkdir -p "$ROS_WS"/maqui_ws
mkdir -p "$ROS_WS"/complete_ws