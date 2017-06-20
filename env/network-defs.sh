#!/bin/sh

###########################################################
# Net configuration
# 
# recordar setear red en cada pc:
#  - connection name: "bendernet"
#  - (general) conectar automaticamente
#  - (ipv4): Method: Manual
#  - (ipv4): address: 192.168.0.X
#  - (ipv4): address: 255.255.255.0
#  - (ipv4): address: 192.168.1.1
#  - (ipv4-routes): Use this connection only for 
#                   resources on this network
###########################################################
 
export UCHILE_NET_IP_BENDER_RED="192.168.0.10"
export UCHILE_NET_IP_BENDER_BLUE="192.168.0.20"
export UCHILE_NET_IP_BENDER_GREEN="192.168.0.30"
export UCHILE_NET_IP_BENDER_GRAY="192.168.0.40"

_uchile_net_parse_ip ()
{
	local ipname
	ipname="$1"
	if [ "$ipname" = "gray" ]; then
		printf "%s" "$UCHILE_NET_IP_BENDER_GRAY"
	elif [ "$ipname" = "blue" ]; then
		printf "%s" "$UCHILE_NET_IP_BENDER_BLUE"
	elif [ "$ipname" = "green" ]; then
		printf "%s" "$UCHILE_NET_IP_BENDER_GREEN"
	elif [ "$ipname" = "red" ]; then
		printf "%s" "$UCHILE_NET_IP_BENDER_RED"
	elif [ "$ipname" = "localhost" ]; then
		printf "%s" "127.0.0.1"
	else
		printf "%s" "$ipname"
	fi
}


# utilizados por ros
export ROS_MASTER_URI="http://$(_uchile_net_parse_ip "$UCHILE_NET_MASTER"):11311"
export ROS_IP="$(_uchile_net_parse_ip "$UCHILE_NET_IP")"
export ROSLAUNCH_SSH_UNKNOWN="1"