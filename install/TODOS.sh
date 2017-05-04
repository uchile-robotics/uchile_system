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


- mail avisando sobre cambio de bender_system
	- agregar comando a ejecutar para actualizar el remote

# SIDE EFFECTS:
# ==============================================
# - agregar opcion para USE_OLD_LAYOUT
# - migrar ramas locas de repos que se migran a github
# - util/pkg_install.bash ... updatear repositorios que lo ocupan
# - reset cache helper. avisar sobre eso.
# - avisar para dejar de usar old system. 



old_git_credential_helper=$(git config --global --get credential.helper)



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
