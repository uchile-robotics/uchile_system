# uchile_system

## Overview


## Instalación del sistema

Ejecutar en terminal (`Ctrl+Alt+T`)

```bash
## Pre-requisitos

# ROS Keys
# Evite instalar la versión full (sudo apt-get install ros-indigo-desktop-full) o alguna de las otras variantes.
# ver: http://wiki.ros.org/indigo/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# actualizar base de software
sudo apt-get update

# ROS: ros-indigo-ros-base
# git/hooks: git python-flake8 shellcheck libxml2-utils python-yaml cppcheck
# install: curl openssl pv python-rosinstall python-pip
# conectividad: openssh-client openssh-server
sudo apt-get install ros-indigo-ros-base git python-flake8 shellcheck libxml2-utils python-yaml cppcheck curl openssl pv python-rosinstall python-pip openssh-client openssh-server


## Instalación

# directorio "sano" y con permisos de escritura.
cd "$HOME"

# descargar uch_system
git clone https://github.com/uchile-robotics/uchile_system.git tmp_repo
cd tmp_repo/install

# Obtener repositorios y crear workspaces
# - ejecutar más de una vez en caso de haber fallado en clonar algún repositorio
# - Esto requiere no tener sourceado ROS en la consola actual ni en bash (revisar .bashrc)
chmod +x ws_installer.bash
./ws_installer.bash

# limpiar
cd "$HOME"
rm -rf ~/tmp_repo


## Habilitar workspace para uso en consola

# sólo usuarios de bash
echo '' >> ~/.bashrc
echo '# UChile ROS Framework settings: location, configs and setup script.' >> ~/.bashrc
echo 'export UCHILE_WS="$HOME"/uchile_ws'        >> ~/.bashrc
echo 'export UCHILE_SHELL_CFG="$HOME"/uchile.sh' >> ~/.bashrc
echo '. "$UCHILE_WS"/system/setup.bash'          >> ~/.bashrc

# sólo usuarios de zsh
echo '' >> ~/.zshrc
echo '# UChile ROS Framework settings: location, configs and setup script.' >> ~/.zshrc
echo 'export UCHILE_WS="$HOME"/uchile_ws'        >> ~/.zshrc
echo 'export UCHILE_SHELL_CFG="$HOME"/uchile.sh' >> ~/.zshrc
echo '. "$UCHILE_WS"/system/setup.zsh'           >> ~/.zshrc

# inicializar rosdep
sudo rosdep init
rosdep update
```
Al terminar la instalación debes reabrir el terminal.


## Configuraciones básicas

En el archivo `$HOME/uchile.sh` se deben pueden configurar aspectos del framework como el robot a utilizar y opciones de red. Pon atención en las variables especificadas en tal archivo, pues deberás modificarlas constantemente.

Al menos, debieras configurar la variable de entorno `UCHILE_ROBOT`, que por defecto es `bender`. Ésta permite seleccionar que overlay de workspaces ROS se utilizarán. Todos los overlays diponibles se encuentran en el directorio `$UCHILE_WS/ros/`. Según el valor escogido, el workspace ROS linkeado proveerá distintos packages, y por lo tanto, requerirá distintos pasos de instalación.


## Configuraciones MUY recomendadas

Estas configuraciones son opcionales, pero se recomiendan para facilitar el desarrollo. Leer con atención y sólo habilitar las realmente deseadas.

Ejecutar en terminal (`Ctrl+Alt+T`)

```bash
# Usuario de Git y ~/.gitconfig global
# - provee usuario, mail, colores y aliases para comandos git.
# - tras copiar el .gitconfig, al menos se debe configurar "name" y "email"!!!
cp -bfS.bkp "$UCHILE_SYSTEM"/templates/default.gitconfig ~/.gitconfig
gedit ~/.gitconfig

# configurar ~/.bash_aliases: esto configura el prompt PS1 para git. 
cp -bfS.bkp "$UCHILE_SYSTEM"/templates/bash_aliases ~/.bash_aliases

# variable utilizada por "rosed" y algunos utilitarios de bender.
echo 'export EDITOR="gedit"' >> ~/.bashrc

# En orden:
# - agrega opción "abrir terminal" al click derecho
# - shell más moderno. permite subdivisiones en cada pestaña.
# - utilitario gráfico para git
sudo apt-get install nautilus-open-terminal terminator gitk

# Trabajar en rama "develop" de cada repositorio
# - si tras correr el comando algún repositorio no está en tal rama,
#   debes cambiarlo manualmente.
#   ej:
#   > cdb soft
#   > git checkout develop
bgit checkout develop

# Herramienta meld para git diffs. (OBS!, puede ser molesta para algunos!)
# - permite ver diffs más bellos.
# - descomentar línea [diff] external del .gitconfig.
sudo apt-get install meld
cp "$UCHILE_SYSTEM"/templates/gitconfig_meld_launcher.py "$HOME"/.gitconfig_meld_launcher.py
```


## Compilación de workspaces

En esta fase es importante el orden de compilación.

El sistema se divide en 5 workspaces, que en orden son: ROS, forks_ws, base_ws, soft_ws y high_ws.

Los pasos a seguir dependerán del robot a utilizar, según la variable `$UCHILE_ROBOT`. En caso de querer utilizar ambos robots, seguir todas las instrucciones. Si sólo se instalará para uno de los robots, seguir las instrucciones correspondientes.


### Instalación de `forks_ws`

Ejecutar en terminal (`Ctrl+Alt+T`)

```bash
# Abrir workspace
cdb forks && cd ..

# Instalar dependencias
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# Compilar
catkin_make
```


### Instalación de `base_ws`

#### base_ws (sólo bender)

Ejecutar en terminal (`Ctrl+Alt+T`)

```bash
# instalar dependencias
cdb base
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# install bender_description
cdb bender_description
bash install/install.sh
bash scripts/update_models.sh

# install bender_base
cdb bender_base
bash install/install.sh

# install bender_head
cdb bender_head
bash install/install.sh

# install bender_joy
cdb bender_joy
bash install/install.sh

# install bender_tts
cdb bender_tts
bash install/install.sh

# install bender_fieldbus
cdb bender_fieldbus
bash install/install.sh

# install bender_sensors
cdb bender_sensors
bash install/install.sh

# install bender_turning_base
cdb uchile_turning_base
bash install/install.sh

# Compilar
cdb base && cd ..
catkin_make
```

#### base_ws (sólo maqui)

Ejecutar en terminal (`Ctrl+Alt+T`)

```bash
# instalar dependencias
cdb base
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# Nada que instalar aún!

# Compilar
cdb base && cd ..
catkin_make
```


### Instalación de `soft_ws`

### instalación común

Ejecutar en terminal (`Ctrl+Alt+T`)

```bash
# instalar dependencias
cdb soft
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# instalar dependencias de speech
cdb uchile_speech_pocketsphinx
bash install/install.sh
```

#### instalación sólo bender

```bash
# instalar dependencias de bender_arm_planning
cdb bender_arm_planning
bash install/install.sh

# instalar dependencias para deep learning
# [AVISO] puede tomar un par de horas !!
# [WARNING] Sólo testeado en consola bash. Puede haber problemas con pip. Ver: https://bitbucket.org/uchile-robotics-die/bender_system/issues/9/importerror-no-module-named
# [NOTA] No instalar no afecta en compilar bender
cdb uchile_perception_utils
bash install/install.sh
```

#### Finalmente ...

Finalmente, ejecutar:

```bash
# Compilar
cdb soft && cd ..
catkin_make
```

### Instalación de `high_ws`

Ejecutar en terminal (`Ctrl+Alt+T`)

```bash
# instalar dependencias
cdb high
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# Compilar
cdb high && cd ..
catkin_make
```

### Configuración del simulador Gazebo

Para configurar una versión adecuada del simulador Gazebo debes seguir la documentación del package [bender_gazebo](https://bitbucket.org/uchile-robotics-die/bender_system/wiki/doc/packages/bender_gazebo.md).
