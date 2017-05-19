

REVISAR LINEAS COMENTADASEN PEPEPR


uchile_system
. commits diferidos
. find string seguir links
- catkin?make ..catkin?build


# - paths de cosas que cambiaron de lugar..
#    ej> $BENDER_WS/soft_ws/bender_tools....
# - hacer un tag pre migracion: v1.9
# - hacer un tag post migracion: v2.0
# - check cosas post migracion en bender
# - ajustar uchile_system
# - pushear commits nuevos y marcar repos como obsoletos
# - mail avisando de esto para el viernes... 
#    - recordar pushear todo el codigo local a rama respetiva! 

# checks migracion:
# - ramas
# - tags
# - downloads, wiki, issues
# - ramas protegidas
# - merge y eliminacion de feat-uchile
# - ramas prolemáticas.. por conversar

bender_base_layer
	# - feat-depfix .. usar catkin link.. chequear. borrar.
	# - feat-headjoy .. borrar?
# uchile_knowledge       # OK
# uchile_tools           # MERGES?
# uchile_common          # MERGES?
# bender_core            # MERGES? 
# maqui_core             # TODO

bender_high_layer
# uchile_high
# bender_bringup
# maqui_bringup

bender_soft_layer    # BORRAR
bender_knowledge     # OK. 0 activas, 0 staled.
bender_hri           # OK. 0 activas, 1 staled (11 meses, gonzalo).
bender_manipulation  # CONVERSAR. 3 activas [feat-skills, feat-grasp, feat-trajectory]. 0 staled.
bender_navigation    # CONVERSAR. 1 activa (feat-navigation, diego), 0 staled
bender_perception    # CONVERSAR. 2 activas [(feat-icp, nico encina), (feat-ssearch, luz)], 1 staled (feat-sift-cloth, luz.. 5 meses) make public?
bender_system

bender_code_graveyard
bender_embedded

bender_page          # .. borrar???, preguntar a luz sobre los archivos almacenados ahí.. check que archivos desde uchile-robotics.github.io no referencien archivos de esto.
bender_tools         # BORRAR




## deps
## ----------------------------------------------------------------------------

# # install python-aiml
# if [ ! -d "$framework_path"/deps/base/knowledge/python-aiml ]; then
# 	mkdir -p "$framework_path"/deps/base/knowledge/
# 	cd "$framework_path"/deps/base/knowledge/
# 	git clone https://github.com/uchile-robotics-forks/python-aiml
# 	cd python-aiml
# 	sudo python setup.py install
# 	cd "$framework_path"
# fi
