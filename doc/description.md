# Descripción de UChile ROS Framework

## Configuraciones y Variables de entorno

Una vez instalado el workspace, por defecto en `$HOME/uchile_ws`, habrá un archivo de configuración `$HOME/uchile.sh` para el usuario.

### Configuraciones del workspace

Tanto `$UCHILE_SYSTEM` como `$UCHILE_SHELL_CFG` deben ser configuradas antes de hacer source del script `setup.bash` o `setup.zsh`. Más información en la [sección correspondiente](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/installation.md#habilitar-workspace-para-uso-en-consola).

- `$UCHILE_SHELL_CFG`: Contiene el path completo al archivo de configuración del workspace.
- `$UCHILE_WS`: Contiene el path completo al workspace.
- `$UCHILE_ROBOT` (bender/maqui/all): Permite seleccionar entre trabajar sólo con código bender, maqui o todo el código.

### Configuraciones de Red

Se configuran en el archivo designado por `$UCHILE_SHELL_CFG`.

- `$UCHILE_NET_ENABLE` (true/false): Activar o no el uso de ROS por RED.
- `$UCHILE_NET_IP`: IP de la máquina. Sólo si `UCHILE_NET_ENABLE=true`.
- `$UCHILE_NET_MASTER`: IP del ROS Master. Sólo si `UCHILE_NET_ENABLE=true`.
- `$UCHILE_NET_WARN` (true/false): Dar un warning en caso de que la consola actual tenga `UCHILE_NET_ENABLE=false`. Se recomienda su uso en el robot!, pues siempre trabajará por red.

### Variables de Ambiente

Al cargar el workspace se exportan muchas variables de entorno. Las principales son:

- `$UCHILE_SYSTEM`: Path completo a éste repositorio. Equivalente a `"${UCHILE_WS}"/system`
- `$UCHILE_ROS_WS`: Path completo al conjunto de workspaces según el robot seleccionado. Equivalente a `"${UCHILE_WS}"/ros/${UCHILE_ROBOT}`
- `$UCHILE_PKGS_WS`: Path completo a carpeta con todos los paquetes ROS, independiente del robot. Equivalente a `"${UCHILE_WS}"/pkgs`

Las demás variables de ambientes pueden ser vistas con el comando `uchile_printenv` o mediante `export | grep UCHILE`.


## Estructura del workspace

Fue diseñado para poder trabajar con múltiples robots en ROS. Actualmente maneja los repositorios de **bender** y **maqui**.

El workspace se divide en los siguientes directorios:

- `system`: Éste repositorio.
- `deps`: Carpeta pensada para almacenar todas las dependencias instalables de los packages ROS utilizados.
- `pkgs`: Contiene todos los repositorios de UChile Robotics @Home. Son packages o metapackages de ROS.
- `ros`: Contiene workspaces ROS con links simbólicos a los verdaderos repositorios ubicados en `pkgs`
- `misc`: Otros repositorios: Código obsoleto, página web, wiki, repositorios no ROS. 

El directorio `pkgs` se divide en 4 espacios de trabajo, que forman una estructura jerárquica, en donde el código de cada uno sólo depende de los repositorios en espacios de menor nivel. En orden son:

- `forks_ws`: Packages ROS con forks utilizados y mantenidos por el equipo. Generalmente, dependencias para los siguientes niveles.
- `base_ws`: Packages ROS que permiten lanzar la interfaz ROS de bajo nivel de un robot: msgs, srvs, tfs, drivers, knowledge y utilitarios.
- `soft_ws`: Packages ROS que proveen capacidades especializadas y construidas sobre `base_ws`: HRI, Manipulación, Navegación y Visión.
- `high_ws`: Packages ROS con la interfaz de alto nivel del robot: bringups, robot skills, máquinas de estado y demostraciones.

La separacion en los 4 workspaces busca 2 objetivos: ordenar la gran cantidad de packages del equipo. Forzar el desacople del código, evitando dependencias innecesarias entre packages.

Por otro lado, el directorio `ros` contiene 3 espacios de trabajo, seleccionados mediante la variable de entorno `UCHILE_ROBOT`

- `ros/bender/`: Links sólo a repositorios utilizados por bender: Comunes + bender específcos.
- `ros/maqui/`: Ídem para maqui.
- `ros/all/`: Links a todos los repositorios.

Cada uno de éstos espacios de trabajo contiene workspaces ROS análogos a la estructura del directorio `pkgs`, pero donde sólo hay links simbólicos.

### base_ws

Contiene los siguientes metapackages:

- uchile_common: Packages comunes y necesarios para todo robot: msgs, srvs, tfs, utilitarios C++ y python.
- uchile_knowledge: Packages para el manejo de la memoria del robot: bases de datos, mapa, diccionarios, etc.
- uchile_tools: Packages con herramientas variadas.
- *{robot}_core*: Packages específicos para cada robot: drivers y simulación.

### soft_ws

Contiene los metapackages: `uchile_navigation`, `uchile_hri`, `uchile_manipulation`, `uchile_perception`.

### high_ws

Contiene el metapackage `uchile_high`, con los packages:

- uchile_skills: Definición e implementación de las robot skills.
- uchile_states: Máquinas de estado básicas, implementadas a partir de las robot skills.
- uchile_robocup: Máquinas de estado de alto nivel, especializadas para las pruebas de la robocup.
- uchile_demos: Máquinas de estado de alto nivel, para realizar comportamientos variados. Útiles en demostraciones.

Además, existe un package extra por cada robot: *{robot}_bringup*. Éste contiene los launchfiles y configuraciones específicas para el robot, que permiten lanzar los nodos de `base_ws` y de `soft_ws`. Entonces, se espera que los packages comunes sean robot agnósticos y permitan ser configuradados a través de los launchfiles que proveen; Idealmente, la configuración debe ser realizada mediante el parameter server de ROS.

