# bender_system


## Overview



## Instalación del sistema

Ejecutar en terminal (`Ctrl+Alt+T`)

```
#!/bin/bash

# Requisitos para los git hooks
sudo apt-get install python-flake8 shellcheck libxml2-utils python-yaml cppcheck curl

# directorio "sano"
cd "$HOME"

# Obtener script de instalación (usar credenciales de bitbucket)
curl --user <username>:<pass> -G https://bitbucket.org/uchile-robotics-die/bender_system/raw/master/bash/bender_ws_installer.bash -d raw > bender_ws_installer.bash

# Dar permisos de ejecución
chmod +x bender_ws_installer.bash

# Obtener repositorios y crear workspaces
./bender_ws_installer.bash

# Hacer source
echo -e '\nsource "$HOME"/bender.sh' >> .bashrc

# Se recomienda setear la siguiente variable
echo 'export EDITOR="gedit"' >> .bashrc

```

## Instalación de `base_ws`

```
#!/bin/bash

# Abrir workspace
bender_cd base

# Instalar dependencias
rosdep install --from-paths . --ignore-src --rosdistro=indigo -y

# Compilar
cd ..
catkin_make

```