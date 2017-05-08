# TODOS:
# - version para maqui en gentoo. 
#   - ROS check
# opcion: USE_OLD_LAYOUT
#


install -> deps
misc/embedded
misc/code_graveyard
/ros/high_ws/src
/ros/soft_ws/src
/ros/base_ws/src
/system
/misc/wiki

- activar hooks


bgit fetch usando credential helper.. evitar introducir contrasenas

- mail avisando sobre cambio de bender_system
	- agregar comando a ejecutar para actualizar el remote



# SIDE EFFECTS:
# ==============================================
# - agregar opcion para USE_OLD_LAYOUT
# - migrar ramas locas de repos que se migran a github
# - util/pkg_install.bash ... updatear repositorios que lo ocupan
# - reset cache helper. avisar sobre eso.
# - avisar para dejar de usar old system. 
# - instrucciones en el README sobre como migrar a nueva versión.
#   - avisar para que recuerden chequear si tienen ramas que no
#     deberían ser borradas. Crear comando utilitario para eso.
# - script para pushear todos sus cambios a ramas respectivas github? peligroso

old_git_credential_helper=$(git config --global --get credential.helper)


# POR MIGRAR
# ====================================================================================
# COMPAT MODE: will work but should be updated to v2.0

bender.sh
network_defs
migrar issues, wiki

# BENDER_WS (COMPAT MODE)
./bender_sim/bender_gazebo/install/install.sh:33:BACKUP_FOLDER="$BENDER_WS"/install/base/sim/gazebo
./bender_hardware/bender_base/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_joy/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_head/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_fieldbus/install/install.sh:16:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_tts/install/settings.bash:21:install_space="$BENDER_WS"/install/base/hardware/tts
./bender_hardware/bender_tts/install/install.sh:16:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_sensors/install/lasers.sh:5:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_sensors/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_common/bender_description/scripts/update_models.sh:19:cd "$BENDER_WS"/base_ws/src/bender_common/bender_description/robots
./bender_common/bender_description/install/install.sh:30:BACKUP_FOLDER="$BENDER_WS"/install/base/common/description
./bender_common/bender_description/install/install.sh:58:tar -xzf "$BACKUP_FILE" --directory "$BENDER_WS"/base_ws/src/bender_common/
./bender_common/bender_tf/scripts/load_calibration:43:    calibration_folder="$BENDER_WS/base_ws/src/bender_common/bender_tf/calibration"
./bender_common/bender_tf/scripts/save_calibration:26:    calibration_folder="$BENDER_WS/base_ws/src/bender_common/bender_tf/calibration"
./bender_knowledge/bender_db/db/download.sh:46:#unzip bender_description.zip -d $BENDER_WS/base_ws/src/bender_common/
./bender_core_tools/bender_turning_base/install/install.sh:13:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hri/bender_speech/install/install.sh:9:# install_space="$BENDER_WS"/install/soft/hri/speech
./bender_hri/bender_speech/install/install.sh:28:install_space="$BENDER_WS"/install/soft/hri/speech
./bender_hri/bender_nlp/install/install.sh:9:install_path="$BENDER_WS/install/soft/hri/nlp"
./bender_manipulation/bender_arm_planning/install/install.sh:32:BACKUP_FOLDER="$BENDER_WS"/install/soft/manipulation/arm_planning
./bender_manipulation/bender_arm_planning/install/install.sh:61:unzip -q -o "$BACKUP_FILE" -d "$BENDER_WS"/soft_ws/src/bender_manipulation/
./bender_navigation/bender_nav/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_perception/bender_person/waving_detector/install_nite.sh:6:BACKUP_FOLDER="$BENDER_WS"/install/soft/perception/person/waving_detector
./bender_perception/bender_person/waving_detector/CMakeLists.txt:2:if(EXISTS "$ENV{BENDER_WS}/install/soft/perception/person/waving_detector")
./bender_perception/bender_person/waving_detector/CMakeLists.txt:5:    set(OPENNI2_DIR "$ENV{BENDER_WS}/install/soft/perception/OpenNI2")
./bender_perception/bender_person/waving_detector/CMakeLists.txt:6:    set(NITE2_DIR "$ENV{BENDER_WS}/install/soft/perception/person/waving_detector/NiTE-2.0.0")
./bender_perception/bender_person/waving_detector/CMakeLists.txt:7:    set(NITE2_LIB "$ENV{BENDER_WS}/install/soft/perception/person/waving_detector/NiTE-2.0.0/Redist/libNiTE2.so")
./bender_perception/bender_perception_utils/install/install_caffe.sh:8:cd "$BENDER_WS"/install/soft
./bender_perception/bender_perception_utils/install/install_caffe.sh:31:echo 'export PYTHONPATH="$BENDER_WS"/install/soft/perception/caffe/python:$PYTHONPATH' >> ~/.bashrc 
./bender_perception/bender_perception_utils/install/install_openface.sh:8:cd "$BENDER_WS"/install/soft
./bender_perception/bender_perception_utils/install/install_openface.sh:30:cd "$BENDER_WS"/install/soft/perception/torch/install/bin
./bender_perception/bender_perception_utils/install/install_openface.sh:36:cd "$BENDER_WS"/install/soft/perception
./bender_perception/bender_perception_utils/install/install_torch.sh:8:cd "$BENDER_WS"/install/soft
./bender_perception/bender_perception_utils/install/reinstall_pip.sh:8:cd "$BENDER_WS"/install/soft

# BENDER_SYSTEM (COMPAT MODE)
./bender_sim/bender_gazebo/install/install.sh:40:	"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!G1MCEZIQ!u2UDNUMPPKODJ0u5-qs_OkA5hodXh3ilwSP8CoYSSD8'
./bender_hardware/bender_tts/install/settings.bash:14:megadown_exe="$BENDER_SYSTEM"/shell/megadown/megadown
./bender_common/bender_description/install/install.sh:37:	"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!39sF0LSJ!HXo1Q1_KaqKwwVNsmUKfr_rV3vcZ_IiQpdUWs8F2IJQ'
./bender_knowledge/bender_db/games/download.sh:4:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!S8VFWKiI!nDWmfGUfCxnRql-GCp3ln37EoSKUHLKlNsUK8mVtU-w'
./bender_knowledge/bender_db/db/download.sh:6:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!Tx0zFKIQ!AB-fAXmORZKt2pyXQIe72Y04VdGwCGn1FM38i0DnCms'
./bender_knowledge/bender_db/db/download.sh:9:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!PwU00D7A!vz-tV60YdWJeJu3hzwH75y6TF2b0WKSuZbcfOam5wXI'
./bender_knowledge/bender_db/db/download.sh:12:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!7lkQQKKR!FfSKOCLo0jiFutwg4H5-oOnepmWszlbLDJAQWcmua3o'
./bender_knowledge/bender_db/db/download.sh:15:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!K8tEDRjZ!eONgpZz0XWghyNeB-HfGseU71ZMlYvqLFVFDoPx2fdQ'
./bender_knowledge/bender_db/db/download.sh:18:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!uo1T1D5R!PdV1SgMHMnEHRqzjOaHnFaz8UrDYpcPVPIrZCsk5RoE'
./bender_knowledge/bender_db/db/download.sh:21:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!O09zFCLL!2AHjxXGVx6G6R2MorhIh-83Ww6FJNOd-cN3vBu4XIcE'
./bender_knowledge/bender_db/db/download.sh:24:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!i5sARZQI!pf3fCLv_ajP7NsLUdJS1_GboxFPI2TmenVB_Ovlz18E'
./bender_knowledge/bender_db/db/download.sh:27:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!O80iVRBY!FJ8e47vlg09d2JSHNYL2rC8bYdKN_MfJD25bUF_wP9M'
./bender_knowledge/bender_db/db/download.sh:30:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!nsNXUaIB!xzUfxutRe6J_WGObw8Yd5IDZ8bee3CBIlK6W9TPLMzU'
./bender_knowledge/bender_db/db/download.sh:36:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!Sh82VaLY!92B99P8G4KKr9Rz6sb4zmj4YLGNWflqs5Ni_Z8IYb48'
./bender_knowledge/bender_db/db/download.sh:39:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!Ph1z1YJR!uYCjgY8wpMT1WsE6qxVXuPV84tpBhT_wH-abppQLQl8'
./bender_knowledge/bender_db/db/download.sh:42:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!7l130Q6Q!4zOzDcrODzLX07TmSp4EgjKb0rTFLRps9N9L-FwOfZU'
./bender_knowledge/bender_db/ros_pkgs/download.sh:8:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!ik0XUKKI!kzyZp2hbg09Ol_Tg1MuFlqxIfR8F01h4HGOKl3lHpos'
./bender_knowledge/bender_db/ros_pkgs/download.sh:11:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!a0930ZYB!sciM89vyDLtrpDpC5_CB-6BCXA6OE1p2Bzq8R6rWeRU'
./bender_knowledge/bender_db/ros_pkgs/download.sh:14:"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!K1F3jRgY!S20epp5617NV7BkEaSaRCblGgZ9MoT2M9n1f_pP6Gck'
./bender_manipulation/bender_arm_planning/install/install.sh:39:	"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!e9NmnD6D!kXfuEhqlOtme87Cb1ZyiIDLI7XBzRdBWikLq0Pbu4KA'
./bender_perception/bender_person/waving_detector/install_nite.sh:14:	"$BENDER_SYSTEM"/shell/megadown/megadown 'https://mega.nz/#!ztExxYgb!qJSSIqIBoGu5jOMOcDHhMsfCGHSN9bQ1G8qgKHDJH2E'


# bender_system
./bender_hardware/bender_base/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_joy/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_head/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_fieldbus/install/install.sh:16:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_tts/install/install.sh:16:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_sensors/install/lasers.sh:5:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_hardware/bender_sensors/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_core_tools/bender_turning_base/install/install.sh:13:source "$BENDER_WS"/bender_system/install/pkg_install.bash
./bender_navigation/bender_nav/install/install.sh:14:source "$BENDER_WS"/bender_system/install/pkg_install.bash



# ERRORES>
# ====================================================================================

# Clonar en «python-aiml»...
# remote: Counting objects: 895, done.
# remote: Total 895 (delta 0), reused 0 (delta 0), pack-reused 895
# Receiving objects: 100% (895/895), 2.82 MiB | 544.00 KiB/s, done.
# Resolving deltas: 100% (589/589), done.
# Comprobando la conectividad… hecho.
# [sudo] password for mpavez: 
# Traceback (most recent call last):
#   File "setup.py", line 70, in <module>
#     setup( **setup_args )
#   File "/usr/lib/python2.7/distutils/core.py", line 111, in setup
#     _setup_distribution = dist = klass(attrs)
#   File "/usr/local/lib/python2.7/dist-packages/setuptools/dist.py", line 320, in __init__
#     _Distribution.__init__(self, attrs)
#   File "/usr/lib/python2.7/distutils/dist.py", line 287, in __init__
#     self.finalize_options()
#   File "/usr/local/lib/python2.7/dist-packages/setuptools/dist.py", line 386, in finalize_options
#     ep.require(installer=self.fetch_build_egg)
#   File "/usr/lib/python2.7/dist-packages/pkg_resources.py", line 2100, in require
#     working_set.resolve(self.dist.requires(self.extras),env,installer)))
#   File "/usr/lib/python2.7/dist-packages/pkg_resources.py", line 620, in resolve
#     dist = best[req.key] = env.best_match(req, ws, installer)
#   File "/usr/lib/python2.7/dist-packages/pkg_resources.py", line 858, in best_match
#     return self.obtain(req, installer) # try and download/install
#   File "/usr/lib/python2.7/dist-packages/pkg_resources.py", line 870, in obtain
#     return installer(requirement)
#   File "/usr/local/lib/python2.7/dist-packages/setuptools/dist.py", line 416, in fetch_build_egg
#     from setuptools.command.easy_install import easy_install
#   File "/usr/local/lib/python2.7/dist-packages/setuptools/command/easy_install.py", line 51, in <module>
#     from setuptools.archive_util import unpack_archive
#   File "/usr/local/lib/python2.7/dist-packages/setuptools/archive_util.py", line 11, in <module>
#     from pkg_resources import ensure_directory, ContextualZipFile
# ImportError: cannot import name ContextualZipFile
