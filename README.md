# UChile ROS Framework

<!--- * [Descripción](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/description.md) -->
* [Instalación](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/installation.md)
<!--- * [Herramientas](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/tools.md) -->
* [TODO List](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/TODO.md)
* <!--- * [Change List](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/tools.md) -->




<!--
## Descripción del Workspace
 * El workspace aún se divide en forks_ws, base_ws, soft_ws y high_ws.
--- base_ws: Contiene metapackages: uchile_common, uchile_tools, uchile_knowledge junto a bender_core/maqui_core, que contienen código equivalente al de bender_hardware, es decir, todas los packages ROS que funcionan a modo de driver o simulación.
--- soft_ws: Contiene metapackages uchile_hri, uchile_manipulation, uchile_navigation y uchile_perception. (Todo idéntico, sólo cambian los nombres).
--- high_ws: Contiene metapackage uchile_high y packages: maqui_bringup, bender_bringup.  Los *_bringup están pensados para almacenar los launchfiles y configs (.yaml, .rviz, ...) específicas para lanzar a cada robot.
 -->

## Recursos Externos

* [Página Oficial de UChile Robotics](http://robotica-uchile.amtc.cl/)
* [Wiki de UChile Robotics @Home](https://github.com/uchile-robotics/uchile_system/wiki)

## Lista de Cambios

### Principales cambios desde la versión 1.9 y tras la migración

Hasta la versión 1.9, todo el código estaba hosteado en bitbucket de manera privada y diseñado para sólo 1 robot, bender.

#### Repositorios

Los principales cambios en relación a los repositorios se pueden resumir en:

* Todos los repositorios están hosteados en GitHub, bajo 3 organizaciones distintas:
    - uchile-robotics: repositorios principales
    - uchile-robotics-forks: forks de repos externos
    - uchile-robotics-graveyard: repos y código obsoleto
* Por temas legales, algunos repositorios se mantienen privados.
* Cada repositorio tiene su propio issue tracker y tablero de proyecto.
* Existe sólo una [wiki](https://github.com/uchile-robotics/uchile_system/wiki).
* Ya no existen submódulos en repositorios
* Se migró todo el historial de los repositorios antiguos (dentro de lo posible)
* Repositorios bender/maqui específicos usan prefijos `bender_*` o `maqui_*`.

#### Workspace

* Workspace ubicado en `$HOME/uchile_ws`.
* Archivo de configuración ubicado en `$HOME/uchile.sh`.
* Workspace alberga 3 overlays de workspaces ROS distintos, manejados por la variable de entorno `$UCHILE_ROBOT`. En cada caso se hará source del archivo `high_ws/devel/setup.sh`. Los packages ROS contenidos en cada caso son sólo links simbólicos a los packages ubicados en `$UCHILE_WS/pkgs`
    - Si `$UCHILE_ROS_WS == bender`: Source de workspaces en `$UCHILE_WS/ros/bender`. Contiene packages comunes y bender específicos.
    - Si `$UCHILE_ROS_WS == maqui`: Source de workspaces en `$UCHILE_WS/ros/maqui`. Contiene packages comunes y maqui específicos.
    - Si `$UCHILE_ROS_WS == all`: Source de workspaces en `$UCHILE_WS/ros/all`. Contiene todos los packages.
* Packages ROS y herramientas para la bash/zsh comunes a ambos robots, cambian prefijos de nombre desde `bender_*` a `uchile_*`
* Las variables de entorno cambian prefijos de nombre desde `BENDER_*` a `UCHILE_*`.
* El package antes llamado *bender_core*, que contenía los launchfiles para bender, ahora se llama *bender_bringup* y se encuentra en el *high_ws*.
