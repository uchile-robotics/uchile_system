
# TODOs


## Varios

* configuración recomendada de cache helper `old_git_credential_helper=$(git config --global --get credential.helper)`
* assets/bender.machine, env/network-defs.sh and env/env-loader.sh
* hay hooks no funcionando?
* linters

##  Migración

* catkin?make ..catkin?build
* hacer un tag pre migracion: v1.9
* hacer un tag post migracion: v2.0
* marcar repos como obsoletos


### repositorios obsoletos

* https://github.com/uchile-robotics/maqui_core y maqui_skills
* https://github.com/uchile-robotics/pepper_apps
* https://github.com/uchile-robotics/robot_skills


## Install

### python-aiml

```bash
if [ ! -d "$framework_path"/deps/base/knowledge/python-aiml ]; then
	mkdir -p "$framework_path"/deps/base/knowledge/
	cd "$framework_path"/deps/base/knowledge/
	git clone https://github.com/uchile-robotics-forks/python-aiml
	cd python-aiml
	sudo python setup.py install
	cd "$framework_path"
fi
```

### en robot maqui

Instalador para el robot


## Pepper


* joysticks se interfieren entre si!
    * trade off entre tener mensajes al usar autorepeat rate de 5hz vs. tener que mover la palanca

###  Modificaciones navegación

```
    modificado: bender_nav/config/amcl/amcl_diff.yaml
    modificado: bender_nav/config/bender_nav/goal_server.yaml
    modificado: bender_nav/config/gmapping/gmapping.yaml
    modificado: bender_nav/config/move_base/costmap_common.yaml
    modificado: bender_nav/config/move_base/local_costmap.yaml
    modificado: bender_nav/config/move_base/move_base.yaml
    
    modificado: bender_nav/launch/localization/amcl.launch.xml
    modificado: bender_nav/launch/localization/gmapping.launch.xml
    modificado: bender_nav/launch/goal_server.launch.xml
    modificado: bender_nav/launch/move_base.launch.xml
    modificado: bender_nav/launch/navigation.launch
    modificado: bender_nav/launch/slam.launch
```
