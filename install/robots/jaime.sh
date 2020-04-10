#!/bin/bash
#
# List of repositories to consider when linking the workspace
#
# List items MUST HAVE a single slash!. Not more, not less.

repositories=(

    # forks
    "forks_ws/navigation"
    "forks_ws/dynamixel_motor"
    "forks_ws/usb_cam"
    "forks_ws/jaime"
    #"forks_ws/open_ptrack"

    # base
    "base_ws/uchile_common"
    "base_ws/uchile_knowledge"
    "base_ws/uchile_tools"
    "base_ws/jaime_core"

    # soft
    "soft_ws/uchile_hri"
    "soft_ws/uchile_navigation"
    #"soft_ws/uchile_perception"

    # high
    "high_ws/uchile_high"
    "high_ws/jaime_bringup"

)
