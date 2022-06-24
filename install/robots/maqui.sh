#!/bin/bash
#
# List of repositories to consider when linking the workspace
#
# List items MUST HAVE a single slash!. Not more, not less.

repositories=(

    "forks_ws/navigation"
    "forks_ws/rosaria"
    "forks_ws/dynamixel_motor"
    "forks_ws/urg_node"
    "forks_ws/usb_cam"
    "forks_ws/pepper"
    #"forks_ws/open_ptrack"
    "forks_ws/ChatterBot"
    "forks_ws/chatterbot-corpus"
    "forks_ws/yolov3-detector"
    "forks_ws/roboticsgroup_gazebo_plugins"
    "forks_ws/CenterNet"
    "forks_ws/humanoid_msgs"

    # base
    "base_ws/uchile_common"
    "base_ws/uchile_knowledge"
    "base_ws/uchile_tools"
    "base_ws/maqui_core"
    "base_ws/bender_core"

    # soft
    "soft_ws/uchile_hri"
    "soft_ws/uchile_navigation"
    "soft_ws/uchile_manipulation"
    "soft_ws/uchile_perception"

    # high
    "high_ws/uchile_high"
    "high_ws/maqui_bringup"
)
