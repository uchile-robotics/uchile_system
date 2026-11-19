#!/bin/bash
# List every CAN interface on the HOST with its USB bus address, state and bitrate.

found=0
for iface in $(ip -br link show type can | awk '{print $1}'); do
    found=1
    bus=$(ethtool -i "$iface" 2>/dev/null | awk '/bus-info/ {print $2}')
    state=$(ip -br link show "$iface" | awk '{print $2}')
    rate=$(ip -details link show "$iface" | grep -oP 'bitrate \K\d+')
    echo "Interface $iface -> USB port ${bus:-?}  state=${state}  bitrate=${rate:-unset}"
done

[ "$found" -eq 0 ] && echo "No CAN interfaces found (check: lsusb | grep 1d50)"
exit 0
