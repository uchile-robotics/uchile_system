# UChile System, NERO (AgileX 7-DoF arm)

Docker (ROS 2 **Jazzy**) environment to run the [AgileX NERO](https://global.agilex.ai/) robotic arm with the official driver [`agx_arm_ros` (branch `ros2`)](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2) and the [`pyAgxArm`](https://github.com/agilexrobotics/pyAgxArm) SDK.

It is a standalone robot config, so it can run on its own or next to another robot's containers (e.g. Bender) on the same PC.

## Table of contents

- [UChile System, NERO (AgileX 7-DoF arm)](#uchile-system-nero-agilex-7-dof-arm)
  - [Table of contents](#table-of-contents)
  - [How it works](#how-it-works)
  - [Directory structure](#directory-structure)
  - [1. Host installation (once per PC)](#1-host-installation-once-per-pc)
  - [2. Build the image](#2-build-the-image)
  - [3. Configuration](#3-configuration)
  - [4. Deployment on the real robot](#4-deployment-on-the-real-robot)
  - [5. Usage](#5-usage)
    - [Shortcuts](#shortcuts)
    - [Launch files (full form)](#launch-files-full-form)
    - [Motion commands](#motion-commands)
    - [Services](#services)
    - [Feedback topics](#feedback-topics)
    - [Without the real arm](#without-the-real-arm)
  - [Development](#development)
  - [Troubleshooting](#troubleshooting)

## How it works

```
 NERO arm ──CAN bus──► AgileX USB-CAN module ──USB──► host PC
                                                     │  kernel driver gs_usb
                                                     ▼
                                   SocketCAN interface "can_nero" @ 1 Mbit/s
                                   (renamed + activated by udev rule 99-nero.rules)
                                                     │  network_mode: host
                                                     ▼
                         container nero_core: agx_arm_ctrl (pyAgxArm) + MoveIt 2
```

- The CAN module is **not** a `/dev/ttyUSB*` / `/dev/ttyACM*` device. It is a **network interface** (`can0`, `can1`...). Because of that, the container has no `devices:` entry for the arm. It sees the host's `can_nero` through `network_mode: host`, and gets `NET_ADMIN` so it can also bring the interface up itself.
- The interface must be **activated on the host**: bitrate 1 Mbit/s, state `UP`. The udev rule installed by `config.sh` does this automatically every time the module is plugged in, and gives it the stable name `can_nero`.

## Directory structure

```bash
nero
├── config
│   └── config.sh                 # host setup entry point (calls core/config/config.sh)
├── core
│   ├── config
│   │   ├── can_scripts
│   │   │   ├── can_up.sh         # manual CAN activation (fallback for the udev rule)
│   │   │   └── find_can_ports.sh # list CAN interfaces, USB port, state, bitrate
│   │   ├── config.sh             # loads gs_usb, installs can-utils, installs udev rules
│   │   ├── cyclonedds.xml
│   │   ├── nero_aliases.sh       # nero_* shortcuts available inside the container
│   │   └── udev_scripts
│   │       ├── 99-nero.rules
│   │       ├── create_udev_rules.sh
│   │       └── delete_udev_rules.sh
│   ├── docker-compose.yml
│   └── Dockerfile
├── docker-compose.yml            # <- use this one
└── README.md
```

## 1. Host installation (once per PC)

Requirements: Docker with the [linux post-install steps](https://docs.docker.com/engine/install/linux-postinstall) done (see the [main README](../README.md)).

```bash
mkdir -p ~/uchile_robotics && cd ~/uchile_robotics
git clone https://github.com/uchile-robotics/uchile_system.git   # skip if already cloned
source ~/uchile_robotics/uchile_system/nero/config/config.sh
```

`config.sh` does the following:

1. Loads the `gs_usb` kernel module and registers it to load at boot (`/etc/modules-load.d/nero-gs_usb.conf`).
2. Installs `can-utils`, `ethtool` and `iproute2` on the host if they are missing.
3. Installs `99-nero.rules` into `/etc/udev/rules.d/`.

> The scripts assume the repo lives in `~/uchile_robotics/uchile_system`, same as the other robots.

Then **plug in (or re-plug) the USB-CAN module** and check the result:

```bash
lsusb | grep 1d50                     # ... ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
ip -details link show can_nero        # must show state UP ... bitrate 1000000
```

To uninstall the rule: `bash ~/uchile_robotics/uchile_system/nero/core/config/udev_scripts/delete_udev_rules.sh`.

## 2. Build the image

```bash
cd ~/uchile_robotics/uchile_system/nero
docker compose build                 # first build takes a while (MoveIt 2)
```

The image (`uchile_robotics/nero_core`, based on `osrf/ros:jazzy-desktop`) contains:

- `pyAgxArm` (installed with pip into the system python), plus `python-can`, `numpy` and `scipy`.
- `agx_arm_ros` with its `agx_arm_urdf` submodule, built with `colcon build --symlink-install` in `/home/nero_ws`. It contains `agx_arm_ctrl`, `agx_arm_msgs`, `agx_arm_description` and `agx_arm_moveit`.
- `ros2_control`/`ros2_controllers`, MoveIt 2, `rviz2`, `can-utils`, and CycloneDDS as the RMW.

**Reproducible builds.** By default the image builds from the latest `ros2` / `master` branches upstream. To deploy a known-good version, pin the commits:

```bash
docker compose build \
  --build-arg AGX_ARM_ROS_REF=<commit-sha> \
  --build-arg PYAGXARM_REF=<commit-sha>
```

You can also set them under `build.args` in `docker-compose.yml`. To update to the newest upstream: `docker compose build --no-cache`.

## 3. Configuration

All arm parameters live in the `environment:` section of `nero/docker-compose.yml`. The `nero_*` shortcuts read them:

| Variable | Default | Meaning |
|---|---|---|
| `NERO_CAN_PORT` | `can_nero` | CAN interface (must match the udev rule name) |
| `NERO_EFFECTOR` | `none` | `none`, `agx_gripper`, `revo2`, `revo2_pro`, `revo2_touch` |
| `NERO_REVO2_SIDE` | `left` | `left` / `right` (only for `revo2*` hands) |
| `NERO_SPEED_PERCENT` | `0` | Motion speed 1–100 %. `0` keeps the SDK default |
| `NERO_TCP_OFFSET` | `[0.0, …, 0.0]` | TCP offset `[x, y, z, rx, ry, rz]` (m / rad). All values must be floats |

After changing them, recreate the container with `docker compose up -d`. No rebuild is needed.

Other driver parameters (`auto_enable`, `fast_mode`, `fw_version`, `pub_rate`, `enable_timeout`, `gripper_default_effort`, `control_enabled`, `namespace`, `log_level`) can be appended to any shortcut, e.g. `nero_driver auto_enable:=false`. The upstream [launch parameters table](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/README_EN.md#launch-parameters) documents them all.

**DDS:** `core/config/cyclonedds.xml` only peers with `localhost`, so the arm is visible to every container on the same PC that uses CycloneDDS with a localhost peer (Bender and Jaime do). To talk to other machines, add `<Peer Address="host.local"/>` entries, as in `bender/core/config/cyclonedds.xml`.

**Several CAN adapters on one PC:** the udev rule matches any `1d50:606f` adapter. If there is more than one (e.g. two arms), pin the rule to each module's serial number as explained in `99-nero.rules`.

## 4. Deployment on the real robot

1. Mount the arm firmly and keep the workspace clear. Connect the 24 V power and the CAN cable to the USB-CAN module, and plug the module into the PC.
2. Check the CAN bus on the host:
   ```bash
   bash ~/uchile_robotics/uchile_system/nero/core/config/can_scripts/find_can_ports.sh
   candump can_nero            # with the arm powered, frames must stream continuously (Ctrl+C)
   ```
   If `can_nero` does not exist or is `DOWN`, run `bash .../can_scripts/can_up.sh` (or `nero_can_up` inside the container).
3. Start the container (allow X11 first if you will use RViz):
   ```bash
   xhost +local:root
   cd ~/uchile_robotics/uchile_system/nero
   docker compose up -d --remove-orphans
   docker exec -it nero_core bash
   ```
4. Inside the container, start the driver. With the default `auto_enable:=true` **the arm enables its motors on startup**:
   ```bash
   nero_driver          # or nero_rviz / nero_moveit
   ```
5. In a second terminal (`docker exec -it nero_core bash`), check feedback and then move it:
   ```bash
   ros2 topic echo /feedback/arm_status --once
   ros2 topic echo /feedback/joint_states --once
   nero_home
   ```
6. To stop: run `nero_disable` while holding the arm (**it can fall when the motors are disabled**), then Ctrl+C the driver and `docker compose down`.

The `ENTRYPOINT` also starts `sshd` (user `root`, password `docker`), the same as the other robots.

## 5. Usage

All commands below run **inside the container** (`docker exec -it nero_core bash`).

### Shortcuts

Defined in `core/config/nero_aliases.sh`. Extra `name:=value` arguments are passed on to the launch file.

| Command | What it does |
|---|---|
| `nero_driver` | Driver only (headless) |
| `nero_rviz` | Driver + RViz that follows the real arm (`follow:=true control:=false`) |
| `nero_moveit` | Driver + MoveIt 2 + RViz (plan and execute on the real arm) |
| `nero_sim` | MoveIt 2 demo with fake hardware (**no arm needed**) |
| `nero_view` | URDF viewer with joint sliders (**no arm needed**) |
| `nero_enable` / `nero_disable` | Enable / disable motors |
| `nero_home` | Move to the zero position |
| `nero_estop` | Emergency stop (hold current position) |
| `nero_candump` | Dump raw CAN frames from `$NERO_CAN_PORT` |
| `nero_can_up [iface] [bitrate]` | (Re)activate the CAN interface from inside the container (uses `NET_ADMIN`). Normally not needed: the host udev rule does it |
| `nero_follower_mode [iface]` | Switch the arm to follower mode (normal single-arm control on firmware ≥ 1.12). Use it if the arm was left in leader mode (see Troubleshooting). Does not move the arm |

### Launch files (full form)

```bash
ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py        can_port:=can_nero arm_type:=nero effector_type:=none tcp_offset:='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]'
ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py   can_port:=can_nero arm_type:=nero follow:=true control:=false
ros2 launch agx_arm_ctrl start_single_agx_arm_moveit.launch.py can_port:=can_nero arm_type:=nero effector_type:=agx_gripper
```

> Keep **one** control source at a time: the driver topics, MoveIt, or the RViz sliders (`control:=true`). Mixing them makes the arm receive conflicting commands.

### Motion commands

The NERO has 7 joints (`joint1` … `joint7`). Example messages ship in `$NERO_TESTS` (`agx_arm_ros/test/nero`):

```bash
ros2 topic pub -1 /control/move_j sensor_msgs/msg/JointState     "$(cat $NERO_TESTS/test_move_j.yaml)"   # joint motion
ros2 topic pub -1 /control/move_p geometry_msgs/msg/PoseStamped  "$(cat $NERO_TESTS/test_move_p.yaml)"   # point-to-point
ros2 topic pub -1 /control/move_l geometry_msgs/msg/PoseStamped  "$(cat $NERO_TESTS/test_move_l.yaml)"   # linear
ros2 topic pub -1 /control/move_c geometry_msgs/msg/PoseArray    "$(cat $NERO_TESTS/test_move_c.yaml)"   # circular (start, mid, end)
```

Streaming joint control (e.g. from your own node) goes through `/control/joint_states` (`sensor_msgs/JointState`). With an `agx_gripper`, add the `gripper` joint (opening width in m, range [0, 0.1]).

### Services

| Service | Type | |
|---|---|---|
| `/enable_agx_arm` | `std_srvs/SetBool` | enable (`true`) / disable (`false`) |
| `/move_home` | `std_srvs/Empty` | go to zero position |
| `/emergency_stop` | `std_srvs/Empty` | stop and hold position |
| `/control_enable` | `std_srvs/SetBool` | accept / ignore `/control/*` commands |

### Feedback topics

| Topic | Type |
|---|---|
| `/feedback/joint_states` | `sensor_msgs/JointState` (default 200 Hz) |
| `/feedback/tcp_pose` | `geometry_msgs/PoseStamped` |
| `/feedback/arm_status` | `diagnostic_msgs/DiagnosticArray` |
| `/feedback/motor_status` | `diagnostic_msgs/DiagnosticArray` (one entry per joint: temperature, voltage, faults) |
| `/feedback/gripper_status` | `diagnostic_msgs/DiagnosticArray` (with `agx_gripper`) |

### Without the real arm

You can try the image without the arm connected:

```bash
nero_view            # URDF + joint sliders
nero_sim             # MoveIt 2 with fake controllers
```

## Development

To modify the driver, clone it on the host and mount it over the copy inside the image:

```bash
cd ~/uchile_robotics
git clone -b ros2 --recurse-submodules https://github.com/agilexrobotics/agx_arm_ros.git
```

Uncomment the `~/uchile_robotics/agx_arm_ros:/home/nero_ws/src/agx_arm_ros` volume in `docker-compose.yml`, then:

```bash
docker compose up -d
docker exec -it nero_core bash
cd /home/nero_ws && rm -rf build install log && colcon build --symlink-install && source install/setup.bash
```

Your own packages (e.g. a NERO bringup or a Bender integration package) can be mounted the same way into `/home/nero_ws/src/`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `can_nero` missing | `lsusb \| grep 1d50` (module detected?), `lsmod \| grep gs_usb` (driver loaded?), re-plug the module, or run `can_scripts/can_up.sh` |
| Interface exists but is `DOWN` / wrong bitrate | `bash can_scripts/can_up.sh` on the host, or `nero_can_up` in the container |
| `candump can_nero` shows nothing | The arm is not powered, the CAN cable is loose, or CAN H/L are swapped |
| Driver: `Timed out waiting for low-speed feedback` / `Failed to get firmware version`, but `candump` shows data | The arm is in **leader** (teaching) mode: `candump` only shows IDs `151 155 156 157 159 170` and no `2A*`/`25*`/`26*` feedback. Run `nero_follower_mode` (applies immediately, persists across power cycles). On firmware ≥ 1.12 `set_normal_mode()` is a no-op, follower **is** the single-arm mode |
| Driver times out enabling the arm | Check `/feedback/arm_status`, power-cycle the arm, and increase `enable_timeout:=10.0` |
| Driver: `Network is down` / `No such device` | The interface went down on the host after the container started. Bring it up again on the host (no container restart needed) |
| RViz: `could not connect to display` | Run `xhost +local:root` on the host before `docker compose up`, and check `echo $DISPLAY` |
| MoveIt fails parsing numbers | The locale must be English. The image already sets `LC_NUMERIC=en_US.UTF-8` |
| Leader/follower arms on the same CAN module | Set `fw_version:=vXXX` manually (see the upstream README) |

Upstream docs: [README](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/README_EN.md) · [CAN guide](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/docs/CAN_USER_EN.md) · [MoveIt](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/src/agx_arm_moveit/README_EN.md) · [TCP offset](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/docs/tcp_offset/TCP_OFFSET_EN.md) · [Q&A](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/docs/Q%26A.md)
