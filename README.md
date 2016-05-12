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

# para los git y git-hooks
sudo apt-get install git python-flake8 shellcheck libxml2-utils python-yaml cppcheck

# instalación
sudo apt-get install curl openssl pv python-rosinstall

# conectividad
sudo apt-get install openssh-client openssh-server



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
cd "$HOME"
rm -rf ~/tmp_repo


## Habilitar workspace para uso en consola

# Hacer source
echo 'source "$HOME"/bender.sh' >> ~/.bashrc

sudo rosdep init
rosdep update

```
Al terminar la instalación debes reabrir el terminal o ejecutar `$ source "$HOME"/bender.sh`.


## Configuraciones recomendadas

```
#!/bin/bash
# configurar ~/.gitconfig global: usuario, mail, colores y aliases para comandos git.
# - tras copiar el .gitconfig, al menos se debe configurar "name" y "email"!!!
cp -bfS.bkp "$BENDER_SYSTEM"/templates/default.gitconfig ~/.gitconfig
gedit ~/.gitconfig

# configurar ~/.bash_aliases: esto configura el prompt PS1 para git. 
cp -bfS.bkp "$BENDER_SYSTEM"/templates/bash_aliases ~/.bash_aliases

# variable utilizada por "rosed" y algunos utilitarios de bender.
echo 'export EDITOR="gedit"' >> ~/.bashrc

# En orden:
# - agrega opción "abrir terminal" al click derecho
# - shell más moderno. permite subdivisiones en cada pestaña.
sudo apt-get install nautilus-open-terminal terminator
 
```


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