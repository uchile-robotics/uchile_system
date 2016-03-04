# README #


## AUN NO ESTÁ LISTO !!!!


## .


## BASURA abajo


LEEME


1.- Copy ./bender.sh  to  ~/bender.sh and modify it with your current settings.

2.-  Set the following in file '~/.bashrc':

# BENDER ROS INDIGO SETUP
source ~/bender.sh
export EDITOR="gedit" ;


3.- bash_aliases contiene el c'odigo para modificar el prompt
 del bash, para que muestre la rama actual y si est'a sucia.

  Para habilitarlo, basta copiarlo a: ~/.bash_aliases
  Notar que es un archivo oculto!.


---------------------------------------------------


BenderCode-Indigo contiene el código fuente del Robot Bender,
proyecto del laboratorio de robótica de la Universidad
de Chile, FCFM, Depto. de Ingeniería Eléctrica.

# ¡¡¡ NEW Bash Tools !!! #

* `$ bender_cd`: cd al src de bender
* `$ bender_make [bender_pkg | clean | --eclipse | -h ]`: build/clean de packages y .project files para eclipse.
* `$ bender_find_string "hello world"`: encuentra string en el código de bender
* `$ bender_refresh_bash`: permite "re-sourcear" el bash, pero no soluciona el cambio de `BENDER_USE_NETWORK=true/false`
* `$ bender_test_head-icmp`: permite testear conectividad a la cabeza nueva. (hace ping ...)
* `$ bender-set_language [spanish | english]`: permite cambiar el lenguaje del sintetizador de voz
* `$ bender-say "hello world"`: sintetiza con el lenguaje actual 
* `$ bender-say_random_esp"`: sintetiza una frase random en español (sin cambiar el lenguaje)
* `$ bender-say_random_eng"`: sintetiza una frase random en inglés (sin cambiar el lenguaje)
* `$ bender-set_emotion [happy3 | ... ]`: publica tópico con la emoción elegida. Usar autocomplete para ver las disponibles.
* `$ bender-set_mouth_state [speakOn | speakOff ]`: publica tópico con el estado elegido.
* `$ bender-set_neck_yaw <integer>`: publica tópico con el ángulo deseado.


# TODO List

## Problemas más urgentes

Obs: no están ordenados por prioridad

* Corregir "benderlaunch"
* testear installs en un ubuntu limpio

* bender_speech
      * InteractSpeech:
             * ya no tiene timers!, antes tenía en casi de que fallara muchas veces el reconocimiento. Sería buena idea agregar uno en algúna parte del sistema
             * De repente se cae... reproducirlo usando el servicio ``.../request_drink``
      * Automatizar compilación de diccionarios ...
          1. crear utilitario para compilar todos los .jsgf de la carpeta Grammar
          2. modificar recognizer, para que compile todos los diccionarios, de ser necesario.
          3. Crear utilitario que limpie los diccionarios compilados!, dejando sólo las cosas no auto-generadas

*  Automatizar grab suave/fuerte de objetos

## Problemas/Ideas (No Tan Urgentes) ##

* macros para inicializar un timer y setear transiciones adecuadas (como lo utilizado en PersonRecognition.py)
* macros que permitan mover la cabeza en búsqueda de algo.. con un estado de búsqueda agnostico, el que pueda ser pasado como parámetro --> sólo una macro, que genera búsqueda de caras, de operador, de crowds, etc.
* macros/skills para suscripcion y unregister automático de los procesos pessados (como lectura de pointclouds), similar a la idea anterior.

*  agregar archivos de instalación a bender_config, que realicen tareas típicas, pero bien hechas (como lo del acceso a los puertos, p.e), así, todos los instaladores podrán depender de tales utilidades. Además, se podría utilizar bender_all o bender_config para realizar una instalación "total" del sistema.
*  mejorar archivos ``install.sh``, para que no dejen la embarrada en caso de ser ejecutados más de una vez!
* buscar plugins útiles para este repositorio *hooks* (ej: evitar elementos > x MB)
* configuración del micrófono (que no sea necesaria una reconexión random)
* Aria -> voltaje de baterías

# CMakeLists.txt

Generalmente se deberá usar el siguiente layout para agregar algún ejecutable:

```
# compilar source
add_executable(mi_nodo src/mi_nodo.cpp)

# asegurarse de que las dependencias sean compiladas anteriormente
# (útil si se va a ocupar mensajes, servicios u otros)
add_dependencies(mi_nodo ${catkin_EXPORTED_TARGETS})

# linkear dependencias al ejecutable
target_link_libraries(mi_nodo ${catkin_LIBRARIES})
``` 

## links de interés:

* sobre dependencias msg, srv: http://docs.ros.org/groovy/api/catkin/html/howto/format1/cpp_msg_dependencies.html
* sobre dependencias: http://answers.ros.org/question/52744/how-to-specify-dependencies-with-foo_msgs-catkin-packages/
* sobre catkin - explicación desde las bases: g++->make->cmake->catkin: http://jbohren.com/articles/gentle-catkin-intro/