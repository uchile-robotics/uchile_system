## Configuraciones Instalación Ubuntu 
Idioma : English

User : bender
Password : benderrobot
PC-Name : bender-green / bender-navi / bender-chest

## Computer Configuration

Unlock from launcher : libre office, libre excel, libre powerpoint, amazon
sudo add-apt-repository ppa:webupd8team/sublime-text-2
sudo apt-get update
sudo apt-get install sublime-text nautilus-open-terminal terminator vim 
Lock to launcher: terminator sublime 

Allow files to be opened by default on sublime : .launch .xml .py .cpp .h .txt 

Change workspace
sudo apt-get install compizconfig-settings-manager
General Options -> Desktop size -> 3x3

Time settings
Suspend when inactive for : Don't suspend
When the lid is closed : Do nothing

Add Spanish dictionary

### Time synchronzatin between pcs

Install `chrony` on all machines:

```bash
# Install NTP server on 
sudo apt-get update
sudo apt-get install chrony
```

Then, configure each slave machine to listen the master. This assumes the ROS master machine is located at `192.168.0.10`. Modify the chrony `/etc/chrony/chrony.conf` on each slave. Add the following line among the other configured servers (after the `server 3.debian.pool.ntp.org offline minpoll 8` line)

```
server 192.168.0.10 minpoll 0 maxpoll 5 maxdelay .05
```

The master must not be modified.

See also [ROS Network Setup Guide](http://wiki.ros.org/ROS/NetworkSetup#Timing_issues.2C_TF_complaining_about_extrapolation_into_the_future.3F) for information on how to setup chrony.


##Tests before continuing

sudo apt-get install synaptic audacity
Test Microphone sounds in audacity


## Install ros and bender/maqui

* Follow the install steps

### Recommendations
cp -bfS.bkp "$UCHILE_SYSTEM"/templates/default.gitconfig ~/.gitconfig
git config --global user.name 'benderuchile'
git config --global user.email 'bender.devel@gmail.com'
git config --global credential.helper 'cache --timeout=86400'


echo 'export EDITOR="subl"' >> ~/.bashrc

Do not add ''' Configuraciones Menos recomendadas '''

Configurar uchile.sh

HARK :

http://www.hark.jp/wiki.cgi?page=HARK+Installation+Instructions
http://www.hark.jp/wiki.cgi?page=HARK-ROS+Installation+Instructions

Bookmark:
uchile_robocup
uchile_states
bender_skills
uchile_perception
uchile_common
uchile_db/db

## Test 

Gazebo simulator with the corresponding robot

##Download DBs
cdb uchile_db && cd db && ./download.sh 
