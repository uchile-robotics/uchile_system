## Configuraciones Instalación Ubuntu 

- Idioma : English
- User : bender
- Password : benderrobot
- PC-Name : bender-green / bender-navi / bender-chest

## Computer Configuration

```bash
sudo add-apt-repository ppa:webupd8team/sublime-text-2
sudo apt-get update
sudo apt-get install sublime-text nautilus-open-terminal terminator vim gitk 
```


- Software & Updates:
	- Ubuntu Software:
		- Keep default
	- Other Software: Default Config
	- Updates:
		- Automatically Check for updates: Never
		- Notify me of new Ubuntu version: Never
	- Authentication: Keep default
	- Additional Drivers:
		- Do not INSTALL GPU
- Brightness and Lock: 
	- Turn Scren off: Never
	- Lock: OFF
	- Require my password when waking from suspend: Uncheck
- Lock to launcher: terminator sublime
- Unlock from launcher : libre office, libre excel, libre powerpoint, amazon
- Allow files to be opened by default on sublime : .launch, .xml, .py, .cpp, .h, .txt, .md
- Add Spanish dictionary
- Power:
	- Suspend when inactive for : Don't suspend
	- When the lid is closed : Do nothing
	- Show Battery Status in the Menu Bar: When the battery is present
- Security And Privacy
	- Files and Applications:
		- Record file and application usage: UNCHECK
	- Search:
		- When Searching in the dash: UNCHECK include online search results
	- Diagnostics:
		- UNCHECK: Send error reports to canonical
		- UNCKECK: Send occacional system information to canonical
- Bluetooth: Disable
- Backups: DISABLE BACKUPS



## Change workspace

```bash
sudo apt-get install compizconfig-settings-manager unity-tweak-tool
```

- General Options -> Desktop size -> 3x3
- Open unity-tweak-tool and set 3x3 workspaces. Set Home icon on desktop.


## Network Configuration

Create ETHERNET network connection with the following params:

- name: `bender_core`
- IPV4 Settings:
	- Method: Manual
	- Add Address:
		- Address: 192.168.0.MACHINE_NUMBER
		- Netmask: 255.255.255.0
		- Gateway: 192.168.0.1
	- Routes: Check the `Use this connection only for resources on its network`.

The Routes configuration is important, because it allows to use ROS by ethernet AND also use the WiFi!!


## Time settings

### Time & Date

- Location: Santiago
- Set the time: Automatically from the internet



### Time synchronization between pcs

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


## Tests before continuing

```bash
sudo apt-get install synaptic audacity
```

Test Microphone sounds in audacity


## Install ros and bender/maqui

- Follow the install steps

### Recommendations

```bash
cp -bfS.bkp "$UCHILE_SYSTEM"/templates/default.gitconfig ~/.gitconfig
git config --global user.name 'benderuchile'
git config --global user.email 'bender.devel@gmail.com'
git config --global credential.helper 'cache --timeout=86400'
```

```
echo 'export EDITOR="subl"' >> ~/.bashrc
```

Do not add ''' Configuraciones Menos recomendadas '''

Configurar uchile.sh

HARK :
http://www.hark.jp/wiki.cgi?page=HARK+Installation+Instructions
http://www.hark.jp/wiki.cgi?page=HARK-ROS+Installation+Instructions

### Firefox

Bookmark

- uchile_robocup
- uchile_states
- bender_skills
- uchile_perception
- uchile_common
- uchile_db/db

Setup github credentials for bender.devel@gmail.com

## Test 

Gazebo simulator with the corresponding robot

## Download DBs

```
cdb uchile_db && cd db && ./download.sh 
```
