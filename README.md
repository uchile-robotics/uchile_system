# bender_system


## Overview



## Instalación del sistema

Ejecutar en terminal (`Ctrl+Alt+T`)

```
#!/bin/bash

## Pre-requisitos

# ROS baseline
# ver: http://wiki.ros.org/indigo/Installation/Ubuntu
sudo apt-get install ros-indigo-ros-base

# para los git hooks
sudo apt-get install python-flake8 shellcheck libxml2-utils python-yaml cppcheck

# instalación
sudo apt-get install curl openssl pv python-rosinstall 



## Instalación

# directorio "sano" y con permisos de escritura.
cd "$HOME"

# descargar bender_system
git clone https://bitbucket.org/uchile-robotics-die/bender_system.git tmp_repo
cd tmp_repo/install

# Dar permisos de ejecución
chmod +x bender_ws_installer.bash

# Obtener repositorios y crear workspaces
./bender_ws_installer.bash

# limpiar
rm -rf tmp_repo


## Habilitar workspace para uso en consola

cd "$HOME"

# Hacer source
echo 'source "$HOME"/bender.sh' >> .bashrc

# Se recomienda setear la siguiente variable
echo 'export EDITOR="gedit"' >> .bashrc

sudo rosdep init
rosdep update

```

Al terminar la instalación debes reabrir el terminal o ejecutar `$ source "$HOME"/bender.sh`.

## Compilación de workspaces

En esta fase es importante el orden de compilación.

### Instalación de `base_forks`

```
#!/bin/bash

# Abrir workspace
bender_cd forks

# Instalar dependencias
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# Compilar
cd ..
catkin_make

```
### Instalación de `base_ws`

```
#!/bin/bash

# Abrir workspace
bender_cd base

# Instalar dependencias
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# Compilar
cd ..
catkin_make

# Descargar meshes para bender_description
bender_cd bender_description
./scripts/install.sh

```