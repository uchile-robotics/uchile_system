#!/bin/bash
# Manually activate the NERO CAN interface on the HOST.
# Normally NOT needed: the udev rule (99-nero.rules) does this automatically
# on plug-in. Use it if the rule is not installed, or after `ip link set down`.
#
# Usage: can_up.sh [iface=can_nero] [bitrate=1000000]
#   If <iface> does not exist but exactly one CAN interface does (e.g. can0),
#   that one is renamed to <iface>.

IFACE="${1:-can_nero}"
BITRATE="${2:-1000000}"

if ! ip link show "$IFACE" &>/dev/null; then
    mapfile -t CANS < <(ip -br link show type can | awk '{print $1}')
    if [ "${#CANS[@]}" -eq 0 ]; then
        echo "Error: no CAN interface found. Is the USB-CAN module plugged in? (lsusb | grep 1d50)"
        exit 1
    elif [ "${#CANS[@]}" -gt 1 ]; then
        echo "Error: '$IFACE' not found and several CAN interfaces exist: ${CANS[*]}"
        echo "Run: $0 <one_of_them> $BITRATE"
        exit 1
    fi
    echo "Renaming ${CANS[0]} -> $IFACE"
    sudo ip link set "${CANS[0]}" down
    sudo ip link set "${CANS[0]}" name "$IFACE"
fi

sudo ip link set "$IFACE" down
sudo ip link set "$IFACE" type can bitrate "$BITRATE"
sudo ip link set "$IFACE" up

ip -details -br link show "$IFACE"
echo "$IFACE is up at $BITRATE bit/s"
