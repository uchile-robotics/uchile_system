# Ajustes para trabajar Maqui

### Instalación de pynaoqi

Dirigete al siguiente link https://community.ald.softbankrobotics.com/en/resources/software/language/en-gb
En la sección "Pepper SDKs and documentation 2.5.5" descarga "Python 2.7 SDK 2.5.5 Linux 64" y extrae la descarga en /home *

Añadir la librería al Pythonpath (copiar y pegar en terminal):
```bash
cat >> ~/.bashrc <<"EOF"
export PYTHONPATH=${PYTHONPATH}:$HOME/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages
EOF
```

**Nota:  No es necesario instalar en /home, solo preocura añadir la librería al PYTHONPATH*

## Configuraciones de uchile.sh

Añadir las siguientes variables de entorno (puedes copiar y pegar en la terminal)

```bash
cat >> ~/uchile.sh <<"EOF"

## -----------------------------------------------
export robot_ip="ip_de_maqui"
export robot_port="9559"
## -----------------------------------------------

EOF
```

Además debes debes **editar** las siguietntes variables de entorno (dentro de uchile.sh)

```bash
export UCHILE_ROBOT="maqui"
```

y si trabajas constantemente con maqui recomendamos setear:

```bash
export UCHILE_NET_MASTER=$robot_ip
```



