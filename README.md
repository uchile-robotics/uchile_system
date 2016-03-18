# BENDER_SYSTEM


## Overview



## Instalación del sistema




```
#!/bin/bash

# requisitos para los git hooks
sudo apt-get install python-flake8 shellcheck libxml2-utils python-yaml cppcheck

# directorio "sano"
cd "$HOME"

# Obtener script de instalación (usar credenciales de bitbucket)
curl --user <username>:<pass> -G https://bitbucket.org/uchile-robotics-die/bender_system/raw/master/bash/bender_ws_installer.bash -d raw > bender_ws_installer.bash

# darle permisos de ejecución
chmod +x bender_framework_install.bash

# obtener repos y hacer workspaces
./bender_framework_install

# hacer source
echo 'source "$HOME"/bender.sh' >> .bashrc

# para ROS también se recomienda setear la siguiente variable
echo 'export EDITOR="gedit"' >> .bashrc

```
