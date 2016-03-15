# BENDER_SYSTEM


## Overview



## Instalación del sistema

```
#!/bin/bash

# directorio "sano"
cd "$HOME"

# Obtener script de instalación
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