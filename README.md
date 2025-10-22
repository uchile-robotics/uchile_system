# Uchile System

## Table of contents

- [Uchile System](#uchile-system)
  - [Table of contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Image building](#image-building)
  - [Troubleshooting](#troubleshooting)
  - [How to run](#how-to-run)
  - [Development](#development)
    - [Directory structure](#directory-structure)
    - [Component description](#component-description)
    - [Core development](#core-development)
  
## Introduction

The goal of *UChile System* is to provide a solid infrastructure that standardizes the platform where robot software is executed. To achieve this, the system relies on Docker as the main tool for containerization and environment management. If you are still confused on "Why docker?" we recommend that you watch [this video](https://youtu.be/XcJzOYe3E6M?si=Yp4HpKURfwYUSOCh) from "Articulated Robotics".

## Requirements

Before running this project, you must install:

1. [Docker](https://docs.docker.com/engine/install/)

## Installation

```bash
cd $HOME
mkdir uchile_robotics
cd uchile_robotics
git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_system.git
```

### Image building

Build the Docker image with:

```bash
cd ~/uchile_system/bender
sudo docker compose up -d --build --remove-orphans
```

If the image has already been built, you can skip the `--build` flag:

```bash
sudo docker compose up -d --remove-orphans
```

## Troubleshooting

It might happen that you get an error along the lines of `Error response from daemon: error gathering device information while adding custom device "/dev/rplidar": no such file or directory`.

1. Connect all the devices to the machine.
2. If it still doesn't work, the udev rules might not have been set up yet. Run `source ~/uchile_robotics/uchile_system/<robot_name>/config/config.sh`, replacing `<robot_name>` with the robot that you want to use (like `bender` or `jaime`).

## How to run

To run any piece of code you must look up the respective robot package in the organization. Independent of the packages you want to run, everything must be ran inside the docker container you just built. To run any command inside the container: `docker exec -it <container_name> bash`

- [Bender Bringup](https://github.com/uchile-robotics/bender_bringup)
- [Jaime Bringup](https://github.com/uchile-robotics/jaime_bringup)

<!-- (TODO: update jaime_bringup repository and add any other robot that is missing in this list) -->

## Development

Each robot is divided into two main components:

- **Core**: Base configuration, udev rules, communication setup, and environment.
- **Skills**: High-level functionalities and specific robot capabilities.

For detailed information, see:

- [Bender Core README](./bender/core/README.md)
- [Bender Skills README](./bender/skills/README.md)

### Directory structure

When creating a new robot Docker image, follow this directory structure:

```bash
uchile_system
├── robot_1
│   ├── config
│   │   └── config.sh
│   ├── core
│   │   ├── config
│   │   │   ├── config.sh
│   │   │   ├── cyclonedds.xml
│   │   │   └── udev_scripts
│   │   │       ├── 99-bender.rules
│   │   │       ├── create_udev_rules.sh
│   │   │       └── delete_udev_rules.sh
│   │   ├── docker-compose.yml
│   │   ├── Dockerfile
│   │   └── README.md
│   ├── docker-compose.yml
│   └── skills
│       ├── config
│       │   ├── config.sh
│       │   └── cyclonedds.xml
│       ├── docker-compose.yml
│       └── Dockerfile
├── robot_2
...
└── README.md
```

### Component description

- **Udev_scripts**: Ensures that connected devices always have the same name. For example, Bender’s base (the Pioneer P3AT) typically appears as `/dev/ttyUSB0`. With udev rules, it can be renamed to `/dev/pioneer`, guaranteeing the Docker container always finds the correct port.
- **cyclonedds.xml**: Configures the ROS 2 middleware (DDS). Defines how topics, services, and messages are communicated within the system.
- **Dockerfile**: Defines the container environment and installs all necessary dependencies. It behaves like a shell script with extra directives.
- **docker-compose.yml**: Orchestrates multiple containers and defines how they run together. It also allows to define the ports, environment variables and volumes used by a particular container.

### Core development

If needed, you can still add some dependencies to the `bender/core/Dockerfile` file. If you need to modify any repository contents, we recommend that you clone to the `uchile_robotics` directory created above, then add a symlink in the `docker-compose.yml` file with the volumes variable, as shown below:

```bash
cd ~/uchile_robotics
git clone -b feat-jazzy https://github.com/uchile-robotics/bender_bringup.git # just an example
```

Then add the `volumes` variable in the `docker-compose.yml` file.

```yaml
services:
  bender_core:
    image: uchile_robotics/bender_core
    build:
      context: .
      dockerfile: Dockerfile
    container_name: bender_core
    network_mode: host
    devices:
      - /dev/rplidar:/dev/rplidar
      - /dev/pioneer:/dev/pioneer
      - /dev/joy:/dev/joy
      - /dev/event_joy:/dev/event_joy
    volumes:
      - ~/uchile_robotics/bender_bringup:/home/bender_ws/src/bender_bringup
      - ~/uchile_robotics/bender_core:/home/bender_ws/src/bender_core
      - ~/uchile_robotics/uchile_high:/home/bender_ws/src/uchile_high
  ...

```
