#!/bin/bash
# Convenience shortcuts for the AgileX NERO arm (sourced from /root/.bashrc).
# All of them can be overridden from docker-compose.yml (environment:).

export NERO_CAN_PORT="${NERO_CAN_PORT:-can_nero}"
export NERO_EFFECTOR="${NERO_EFFECTOR:-none}"            # none | agx_gripper | revo2 | revo2_pro | revo2_touch
export NERO_REVO2_SIDE="${NERO_REVO2_SIDE:-left}"         # left | right (only for revo2*)
export NERO_TCP_OFFSET="${NERO_TCP_OFFSET:-[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
export NERO_SPEED_PERCENT="${NERO_SPEED_PERCENT:-0}"      # 1-100, 0 = SDK default
export NERO_TESTS=/home/nero_ws/src/agx_arm_ros/test/nero

_nero_args() {
    echo "can_port:=${NERO_CAN_PORT} arm_type:=nero effector_type:=${NERO_EFFECTOR} \
revo2_type:=${NERO_REVO2_SIDE} speed_percent:=${NERO_SPEED_PERCENT}"
}

# Driver only (headless)
nero_driver()  { ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py $(_nero_args) tcp_offset:="${NERO_TCP_OFFSET}" "$@"; }
# Driver + RViz following the real arm (no control from RViz)
nero_rviz()    { ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py $(_nero_args) tcp_offset:="${NERO_TCP_OFFSET}" follow:=true control:=false "$@"; }
# Driver + MoveIt 2 + RViz
nero_moveit()  { ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py $(_nero_args) tcp_offset:="${NERO_TCP_OFFSET}" "$@"; }
# MoveIt demo with fake hardware (NO real arm needed)
nero_sim()     { ros2 launch agx_arm_moveit demo.launch.py arm_type:=nero effector_type:=${NERO_EFFECTOR} revo2_type:=${NERO_REVO2_SIDE} "$@"; }
# URDF viewer with joint sliders (NO real arm needed)
nero_view()    { ros2 launch agx_arm_description display.launch.py arm_type:=nero effector_type:=${NERO_EFFECTOR} "$@"; }

# Services
nero_enable()  { ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: true}"; }
nero_disable() { ros2 service call /enable_agx_arm std_srvs/srv/SetBool "{data: false}"; }
nero_home()    { ros2 service call /move_home std_srvs/srv/Empty; }
nero_estop()   { ros2 service call /emergency_stop std_srvs/srv/Empty; }

# (Re)activate the CAN interface from inside the container (needs cap NET_ADMIN).
# Normally the host udev rule already does this. Usage: nero_can_up [iface] [bitrate]
nero_can_up() {
    local iface="${1:-$NERO_CAN_PORT}" rate="${2:-1000000}"
    ip link set "$iface" down && \
    ip link set "$iface" type can bitrate "$rate" && \
    ip link set "$iface" up && \
    ip -details -br link show "$iface"
}

# Quick CAN sanity check (should print a continuous stream of frames)
nero_candump() { candump "${NERO_CAN_PORT}" "$@"; }
