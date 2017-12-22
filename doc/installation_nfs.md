# Instalación de sistema nfs


* [Prerrequisitos](#Prerrequisitos)
* [Configuracion de red](#Configuracion-de-red)
* [Instalación de UChile ROS Framework](#instalación-de-uchile-ros-framework)
* [Configuraciones](#configuraciones)
* [Compilación de workspaces](#compilación-de-workspaces)
* [Configuración del simulador Gazebo](#configuración-del-simulador-gazebo)

## Prerrequisitos

### Sistemas operativos

Los prerrequisitos del sistema son tener un pc con ubuntu 16.04 desktop el cual sera utilizado como el master del sistema nfs y en el que el desarrollador tiene contacto fisico directo, para esta instalacion este pc se llamara red. Además se debe tener 1 o mas computadores con ubuntu 16.04 server para evitar consumo de recursos en idle debido a la interfaz grafica en el caso de este tutorial estos pcs seran green y blue.

### Conexión lan
Para el desarrollo de esta instalacion es vital que los pcs tengan conexion via ethernet directa mediante ethernet, Las ips ocupadas en la configuracion serán:
```text
192.168.0.20 blue
192.168.0.30 green
192.168.0.40 gray
```
Para setear una Ip estatica mediante consola leer la seccion (Ip estatica mediante consola.

Las cuales se deben encontrar en la variable de ambiente de ``UCHILE_NET_IP_BENDER_<pc_name>`` la cuales se encuentran en el archivo ``$UCHILE_WS/system/env/network-defs.sh`` y para que se lancen al sourcear el workspace es necesario que la variable ``$UCHILE_NET_ENABLE`` que se encuentra en uchile.sh este seteada en true. 


## Configuraciones de red

### Ip estatica mediante consola

Ejecutar en pc cliente,`sudo nano /etc/network/interfaces` y poner el siguente texto.
```text
iface <device> inet static
	address <ip_pc>
	netmask 255.255.255.0
	gateway 192.168.0.255

dns-nameservers 8.8.8.8 8.8.4.4

```
Luego instalar network manager con el siguente comando `sudo apt install network-manager`.

### Configuracion Hostnames ssh
En el pc master ejecutar:
```bash
bash $UCHILE_WS/system/install/uchile_hosts_config.bash
```
Esto guardara los distintos host de bender (green, blue y gray) bajo las siguentes ips
```text
192.168.0.20 blue
192.168.0.30 green
192.168.0.40 gray
```

### Configuracion keys ssh
Para la configuracion de keys para almacenar las claves de ssh se ejecuta en terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)
```bash
ssh-keygen -f ~/.ssh/bender -t rsa #creacion de key bender de tipo rsa
ssh-copy-id -i ~/.ssh/bender bender@UCHILE_NET_IP_<PC_COLOR> #copiar clave de la maquina remota en la key bender.
```
Se debe modificar el archivo de hosts de ssh el cual se encuentra en el directorio  ~/.ssh/config se debe agregar los siguente
```text
Host <pc_color>
        Hostname <pc_color>
        User bender
        IdentityFile ~/.ssh/bender
```
Una vez realizados toda esta configuracion se debe probar si todo funciona correctamente por lo cual se debe abrit un terminal <kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>) y ejecutar el siguente comando:
```bash
ssh <color>
```
El cual si conecta al usuario bender del pc <color> sin pedir contraseña significa que todo funciona correctamente.

### Instalar ROS kinetic en los pcs clientes

Ejecutar en terminal del master (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)

```bash
ssh <color> #logea via ssh en el cliente
```
Luego se procede a seguir los siguentes comandos para instalar ros

```bash
# ROS Keys
# Evite instalar la versión full (sudo apt-get install ros-kinetic-desktop-full) o alguna de las otras variantes.
# ver: http://wiki.ros.org/kinetic/Installation/Ubuntu
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# actualizar base de software
sudo apt-get update

# instalar ROS base
sudo apt-get install ros-kinetic-ros-base curl openssl pv python-rosinstall python-pip python-rosdep

# inicializar rosdep
sudo rosdep init # ignorar si es que falla con "ERROR: default sources list file already exists:..."
rosdep update    # NO EJECUTAR CON SUDO!
```

### Instalar dependencias para system

Ejecutar en terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)

```bash
ssh <color> #logea via ssh en el cliente
```
Luego se procede a seguir los siguentes comandos para instalar dependencias.

```bash
sudo apt-get update
sudo apt-get install git python-flake8 shellcheck libxml2-utils python-yaml cppcheck curl openssl pv python-rosinstall python-pip openssh-client python-termcolor openssh-server python-rosdep
```
## Configuracion de NFS

### Instalacion de NFS
En el master ejecutar en el terminal (<kbd>Ctrl</kbd> + <kbd>Alt</kbd> + <kbd>T</kbd>)

```bash
sudo apt-get update
sudo apt-get install nfs-kernel-server
```
El proceso para instalar en los clientes es similar: 
```bash
ssh <color>
sudo apt-get update
sudo apt-get install nfs-common

#Se debe reinicar para probar que todo funcione correctamente
sudo shutdown -r now
```


## Instalación de UChile ROS Framework
Una vez realizada la instalacion del Framework en el pc maestro [Si no se a realizado click aqui](https://github.com/uchile-robotics/uchile_system/blob/develop/doc/description.md).

### Configuracion de NFS
