# Uchile System

## Table of contents
- [Uchile System](#uchile-system)
  - [Table of contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Requirements](#requirements)
  - [Instalation](#instalation)
    - [Udev rules](#udev-rules)
    - [Image building](#image-building)
  - [How to run](#how-to-run)
  
## Introduction

The objective of *UChile_system* is providing a solid infrastructure that helps standardize the platform in which a piece of robot software is ran. To facilitate this task it is used a tool called *Docker*.

## Requirements

Before running this project, you must install:

1. [Docker](https://docs.docker.com/engine/install/)

## Instalation

```bash
cd ~
mkdir uchile_robotics
cd uchile_robotics
git clone -b feat-jazzy https://github.com/uchile-robotics/bender_bringup.git
git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_system.git
git clone -b feat-jazzy https://github.com/uchile-robotics/bender_core.git
```

### Udev rules

To keep consistency with the devices used by Bender and to avoid that the devices won't let the docker container go up.

```bash
source ~/uchile_robotics/uchile_system/bender/config/udev_scripts/create_udev_rules.sh
```

If you wish to delete these device symlinks, run:

```bash
source ~/uchile_robotics/uchile_system/bender/config/udev_scripts/delete_udev_rules.sh
```

### Image building

Now you will build the docker image

```bash
cd ~/uchile_system/bender
sudo docker compose up -d --build --remove-orphans
```

If you have already run this command before, you can remove the `--build` flag in the command and run it like:

```bash
sudo docker compose up -d --remove-orphans # Run only if you have built the image beforehand
```

## How to run

To run any piece of code you must look up the respective robot package in the organization. Independent of the packages you want to run, everything must be ran inside the docker container you just built. To run any command inside the container: `docker exec -it <container_name> /bin/bash`

- [Bender Bringup](https://github.com/uchile-robotics/bender_bringup)
- [Jaime Bringup](https://github.com/uchile-robotics/jaime_bringup)

<!-- (TODO: update jaime_bringup repository and add any other robot that is missing in this list) -->
