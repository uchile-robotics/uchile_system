#!/bin/bash

#TODO: revisar la ip del master para evitar montar su disco sobre el mismo.
#TODO: hacer un metodo para que quede mas simple esto.
#TODO: Documentar instalador.
#TODO: Crear una nueva variable de ambiente que guarde la ip del nfs.
if "$UCHILE_NET_ENABLE"
then
    printf ""
else
    printf " #######################################\n"
    printf " UCHILE_NET_ENABLE NOT SET IN TRUE \n"
    printf " CHANGE THE UCHILE.SH \n"
    printf " AND RUN THIS SCRIPT AGAIN \n"
    printf " #######################################\n"
    exit 2
fi

if grep -Fq "$UCHILE_NET_IP_BENDER_GREEN" /etc/hosts
then
    printf " #######################################\n"
	printf " HOST GREEN OK \n"
	printf " #######################################\n"
else
    printf " #######################################\n"
	printf " SETTING GREEN HOST WITH IP  \n"
	printf " $UCHILE_NET_IP_BENDER_GREEN" "\n"
    printf " #######################################\n"
    sudo printf " \n" | sudo tee --append /etc/hosts > /dev/null
    sudo echo "$UCHILE_NET_IP_BENDER_GREEN green" | sudo tee --append /etc/hosts > /dev/null
    printf " ALL SET \n"
    printf " PLEASE FOLLOW KEY GENERATOR INSTRUCTIONS \n"
fi

if grep -Fq "$UCHILE_NET_IP_BENDER_BLUE" /etc/hosts
then
    printf " #######################################\n"
    printf " HOST BLUE OK \n"
    printf " #######################################\n"
else
    printf " #######################################\n"
    printf " SETTING BLUE HOST WITH IP  \n"
    printf " $UCHILE_NET_IP_BENDER_BLUE" "\n"
    printf " #######################################\n"
    sudo printf " \n" | sudo tee --append /etc/hosts > /dev/null
    sudo echo "$UCHILE_NET_IP_BENDER_BLUE blue" | sudo tee --append /etc/hosts > /dev/null
    printf " ALL SET \n"
    printf " PLEASE FOLLOW KEY GENERATOR INSTRUCTIONS \n"
fi

if grep -Fq "$UCHILE_NET_IP_BENDER_GRAY" /etc/hosts
then
    printf " #######################################\n"
    printf " HOST GRAY OK \n"
    printf " #######################################\n"
else
    printf " #######################################\n"
    printf " SETTING GRAY HOST WITH IP  \n"
    printf " $UCHILE_NET_IP_BENDER_GRAY" "\n"
    printf " #######################################\n"
    sudo printf " \n" | sudo tee --append /etc/hosts > /dev/null
    sudo echo "$UCHILE_NET_IP_BENDER_GRAY gray" | sudo tee --append /etc/hosts > /dev/null
    printf " ALL SET \n"
    printf " PLEASE FOLLOW KEY GENERATOR INSTRUCTIONS \n"
fi