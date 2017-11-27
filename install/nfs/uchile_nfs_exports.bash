#!/bin/bash

#TODO: revisar la ip del master para evitar montar su disco sobre el mismo.
#TODO: hacer un metodo para que quede mas simple esto.
#TODO: Documentar instalador.
if grep -Fxq "/home/bender/uchile_ws $UCHILE_NET_IP_BENDER_BLUE(rw,sync,no_root_squash,no_subtree_check)" /etc/exports
then
    printf " #######################################\n"
	printf " BLUE IP SET \n"
	printf " #######################################\n"
else
    printf " #######################################\n"
	printf " IP BLUE NOT SET \n"
	printf " #######################################\n"
	printf " SETTING IP $UCHILE_NET_IP_BENDER_BLUE \n"
    printf " \n" | sudo tee --append /etc/exports > /dev/null
    echo "/home/bender/uchile_ws $UCHILE_NET_IP_BENDER_BLUE(rw,sync,no_root_squash,no_subtree_check)" | sudo tee --append /etc/exports > /dev/null
    printf " BLUE SET \n"
fi

if grep -Fxq "/home/bender/uchile_ws $UCHILE_NET_IP_BENDER_GREEN(rw,sync,no_root_squash,no_subtree_check)" /etc/exports
then
    printf " #######################################\n"
	printf " GREEN IP SET \n"
	printf " #######################################\n"
else
    printf " #######################################\n"
	printf " IP GREEN NOT SET \n"
	printf " #######################################\n"
	printf " SETTING IP $UCHILE_NET_IP_BENDER_GREEN \n"
    echo "/home/bender/uchile_ws $UCHILE_NET_IP_BENDER_GREEN(rw,sync,no_root_squash,no_subtree_check)" | sudo tee --append /etc/exports > /dev/null
    printf " GREEN SET \n"
fi

if grep -Fxq "/home/bender/uchile_ws $UCHILE_NET_IP_BENDER_GRAY(rw,sync,no_root_squash,no_subtree_check)" /etc/exports
then
    printf " #######################################\n"
	printf " GRAY IP SET \n"
	printf " #######################################\n"
else
    printf " #######################################\n"
	printf " IP GRAY NOT SET \n"
	printf " #######################################\n"
	printf " SETTING IP $UCHILE_NET_IP_BENDER_GRAY \n"
    echo "/home/bender/uchile_ws $UCHILE_NET_IP_BENDER_GRAY(rw,sync,no_root_squash,no_subtree_check)" | sudo tee --append /etc/exports > /dev/null
    printf " GRAY SET \n"
fi


printf " ALL SET \n"