# Instalación de sistema nfs


* [Prerrequisitos](#Prerrequisitos)
* [Configuracion de red](#Configuracion-de-red)
* [Instalación de UChile ROS Framework](#instalación-de-uchile-ros-framework)


## Prerrequisitos

### Sistemas operativos

Los prerrequisitos del sistema son tener un pc con ubuntu 16.04 desktop el cual sera utilizado como el master del sistema nfs y en el que el desarrollador tiene contacto fisico directo, para esta instalacion este pc se llamara red. Además se debe tener 1 o mas computadores con ubuntu 16.04 server para evitar consumo de recursos en idle debido a la interfaz grafica en el caso de este tutorial estos pcs seran green y blue.

### Conexión lan
Para el desarrollo de esta instalacion es vital que los pcs tengan conexion via ethernet directa mediante ethernet, Las ips ocupadas en la configuracion serán:
```text
192.168.0.20 blue
192.168.0.30 green
192.168.0.40 gray
```
Para setear una Ip estatica mediante consola leer la seccion (Ip estatica mediante consola.

Las cuales se deben encontrar en la variable de ambiente de ``UCHILE_NET_IP_BENDER_<pc_name>`` la cuales se encuentran en el archivo ``$UCHILE_WS/system/env/network-defs.sh`` y para que se lancen al sourcear el workspace es necesario que la variable ``$UCHILE_NET_ENABLE`` que se encuentra en uchile.sh este seteada en true. 


## Configuraciones de red

### Ip estatica mediante consola

Ejecutar en pc cliente,`sudo nano /etc/network/interfaces` y poner el siguente texto.
```text
iface <device> inet static
	address <ip_pc>
	netmask 255.255.255.0
	gateway 192.168.0.255

dns-nameservers 8.8.8.8 8.8.4.4

```
Luego instalar network manager con el siguente comando `sudo apt install network-manager`.

### Configuracion Hostnames ssh
En el pc master ejecutar:
```bash
bash $UCHILE_WS/system/install/uchile_hosts_config.bash
```
Esto guardara los distintos host de bender (green, blue y gray) bajo las siguentes ips
```text
192.168.0.20 blue
192.168.0.30 green
192.168.0.40 gray
```

### Configuracion keys ssh
Para la configuracion de keys para almacenar las claves de ssh se ejecuta en terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)
```bash
ssh-keygen -f ~/.ssh/bender -t rsa #creacion de key bender de tipo rsa
ssh-copy-id -i ~/.ssh/bender bender@UCHILE_NET_IP_<PC_COLOR> #copiar clave de la maquina remota en la key bender.
```
Se debe modificar el archivo de hosts de ssh el cual se encuentra en el directorio  ~/.ssh/config se debe agregar los siguente
```text
Host <pc_color>
        Hostname <pc_color>
        User bender
        IdentityFile ~/.ssh/bender
```
Una vez realizados toda esta configuracion se debe probar si todo funciona correctamente por lo cual se debe abrit un terminal <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>) y ejecutar el siguente comando:
```bash
ssh <color>
```
El cual si conecta al usuario bender del pc <color> sin pedir contraseña significa que todo funciona correctamente.

### Instalar ROS melodic en los pcs clientes

Ejecutar en terminal del master (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)

```bash
ssh <color> #logea via ssh en el cliente
```
Luego se procede a seguir los siguentes comandos para instalar ros

```bash
# ROS Keys
# Evite instalar la versión full (sudo apt-get install ros-melodic-desktop-full) o alguna de las otras variantes.
# ver: http://wiki.ros.org/melodic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# actualizar base de software
sudo apt-get update

# instalar ROS base
sudo apt-get install ros-melodic-ros-base curl openssl pv python-rosinstall python-pip python-rosdep

# inicializar rosdep
sudo rosdep init # ignorar si es que falla con "ERROR: default sources list file already exists:..."
rosdep update    # NO EJECUTAR CON SUDO!
```

### Instalar dependencias para system

Ejecutar en terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)

```bash
ssh <color> #logea via ssh en el cliente
```
Luego se procede a seguir los siguentes comandos para instalar dependencias.

```bash
sudo apt-get update
sudo apt-get install git python-flake8 shellcheck libxml2-utils python-yaml cppcheck curl openssl pv python-rosinstall python-pip openssh-client python-termcolor openssh-server python-rosdep
```
## Configuracion de NFS

### Instalacion de NFS
En el master ejecutar en el terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)

```bash
sudo apt-get update
sudo apt-get install nfs-kernel-server
```
El proceso para instalar en los clientes es similar: 
```bash
ssh <color>
sudo apt-get update
sudo apt-get install nfs-common

#Se debe reinicar para probar que todo funcione correctamente
sudo shutdown -r now
```


## Instalación de UChile ROS Framework
Una vez realizada la instalacion del Framework en el pc maestro si no se a realizado [click aqui](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/description.md).

### Habilitar workspace para uso en consola
En cada caso puedes copiar el bloque de código directo en la terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>).

### Sólo usuarios de bash
```bash
ssh <color>

cat >> ~/.bashrc <<"EOF"

#!/bin/sh
#
# Shell settings for the uchile ROS framework.
#
# Do not source this file into your *rc files.
# This file is automagically sourced whenever a new shell is opened.
#
#
# To enable the UChile ROS Framework, you must setup the following on your rc file:
# 
# # UChile workspace location
# export UCHILE_WS=<FRAMEWORK_PATH>
#
# # workspace configuration file
# export UCHILE_SHELL_CFG="$HOME"/uchile.sh
#
# # source the setup file
# . "$UCHILE_WS"/system/setup.bash  # (on .bashrc : bash only)
# . "$UCHILE_WS"/system/setup.zsh   # (on .zshrc  : zsh only )
#
#

## UCHILE ROS FRAMEWORK SETTINGS
## ==========================================

# robot settings
# ------------------------------------------- 

# available: bender, maqui, all
export UCHILE_ROBOT="bender"


# networking settings
# -------------------------------------------
# available IP names are:
# - red / green / blue / gray

# Your IP address or name
export UCHILE_NET_IP="<color>"

# ROS MASTER IP address or name
export UCHILE_NET_MASTER="red"

# Enable ROS networking (true/false)
export UCHILE_NET_ENABLE=true

# (dis)plays a warning when working in offline mode (true/false)
export UCHILE_NET_WARN=true

export CLIENT=true

EOF

cat >> ~/.bashrc <<"EOF"

## -----------------------------------------------
## UCHILE ROS FRAMEWORK Settings

# workspace location
export UCHILE_WS="$HOME"/uchile_ws

# settings file location
export UCHILE_SHELL_CFG="$HOME"/uchile.sh

# UChile Robotics Framework for BASH
# comment this line to prevent sourcing the framework
. "$UCHILE_WS"/system/setup.bash
## -----------------------------------------------

EOF
```

Ejecutar lo siguiente en un nuevo terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)


### Instalación de `forks_ws`

```bash
ssh <color>
# instalar dependencias
cdb forks && rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

```

### Instalación de `base_ws`

#### base_ws (común)

```bash
ssh <color>
# instalar dependencias
cdb base && rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# install bender_description
cdb bender_description && bash install/install.sh
cdb bender_description && bash scripts/update_models.sh

# install bender_base
cdb bender_base && bash install/install.sh

# install bender_head
cdb bender_head && bash install/install.sh

# install bender_joy
cdb bender_joy && bash install/install.sh

# install bender_tts
cdb bender_tts && bash install/install.sh

# install bender_fieldbus
cdb bender_fieldbus && bash install/install.sh

# install bender_sensors
cdb bender_sensors && bash install/install.sh

# install uchile_turning_base
cdb bender_turning_base && bash install/install.sh
```

### Instalación de `soft_ws`

#### soft_ws

```bash
ssh <color>
# instalar dependencias
cdb soft && rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# instalar dependencias de speech
cdb uchile_speech_pocketsphinx && bash install/install.sh
sudo apt-get install python-alsaaudio

# instalar dependencias de bender_arm_planning
cdb bender_arm_planning && bash install/install.sh

# instalar dependencias de bender_perception
cdb uchile_perception_utils && bash install/install_nite.sh

# instalar dependencias para deep learning
# [AVISO] puede tomar un par de horas !!
# [WARNING] Sólo testeado en consola bash. Puede haber problemas con pip. Ver: https://bitbucket.org/uchile-robotics-die/bender_system/issues/9/importerror-no-module-named
# [NOTA] No instalar no afecta en compilar bender
cdb uchile_perception_utils && bash install/install.sh
```

### Instalación de `high_ws`

#### high_ws (común)

```bash
ssh <color>
# instalar dependencias
cdb high && rosdep install --from-paths . --ignore-src --rosdistro=indigo -y
sudo apt-get install python-aiml

```
