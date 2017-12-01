#!/bin/bash

#TODO: revisar la ip del master para evitar montar su disco sobre el mismo.
#TODO: hacer un metodo para que quede mas simple esto.
#TODO: Documentar instalador.
#TODO: Crear una nueva variable de ambiente que guarde la ip del nfs.
#TODO: que pruebe conecciones trate de modificarlo

if ssh green -t "grep -Fxq '$UCHILE_NET_IP_BENDER_RED:/home/bender/uchile_ws /home/bender/uchile_ws nfs auto,nofail,noatime,nolock,intr,tcp,actimeo=1800 0 0' /etc/fstab"
then
    printf " #######################################\n"
    printf " ALL SET \n"
    printf " #######################################\n"
else
    printf " #######################################\n"
    printf " NFS MASTER IP NOT SET IN THE FSTAB \n"
    printf " #######################################\n"
    printf " SETTING IP " "$UCHILE_NET_IP_BENDER_RED" "\n"
    ssh green -t "printf ' \n' | sudo tee --append /etc/fstab > /dev/null"
    ssh green -t "echo '$UCHILE_NET_IP_BENDER_RED:/home/bender/uchile_ws /home/bender/uchile_ws nfs auto,nofail,noatime,nolock,intr,tcp,actimeo=1800 0 0' | sudo tee --append /etc/fstab > /dev/null"
    printf " ALL SET \n"
    printf " PLEASE RESTART TO SEE THE CHANGES \n"
fi