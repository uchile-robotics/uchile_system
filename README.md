# Uchile System

## Table of contents
- [Uchile System](#uchile-system)
  - [Table of contents](#table-of-contents)
  - [Introduction](#introduction)
  - [Requirements](#requirements)
  - [Instalation](#instalation)
    - [Steps to create the `.netrc` file](#steps-to-create-the-netrc-file)
    - [Udev rules](#udev-rules)
    - [Image building](#image-building)
  - [How to run](#how-to-run)
  
## Introduction

The objective of *uchile_system* is providing a solid infrastructure that helps standarize the platform in which a piece of robot software is ran. To facilitate this task it is used a tool called *Docker*.

## Requirements

Before running this proyect, you must install:

1. [Docker](https://docs.docker.com/engine/install/)

## Instalation

```bash
cd ~
mkdir uchile_robotics
cd uchile_robotics
git clone -b feat-jazzy https://github.com/uchile-robotics/bender_bringup.git
git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_system.git
```

To prevent rate limit errors (HTTP 429) when running the `rosdep` command inside the Docker container, you need to authenticate requests to GitHub by creating a `.netrc` file.

### Steps to create the `.netrc` file

1. Generate a GitHub Personal Access Token (PAT) with minimal public_repo permissions. You can follow this official guide to create the token: [Creating a personal access token](https://docs.github.com/es/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens#creating-a-personal-access-token-classic).
2. Create the `.netrc` file with the following content, replacing `<your_github_token>` with your generated token.

```bash
printf "machine raw.githubusercontent.com\nlogin GITHUB_TOKEN\npassword <your_github_token>" > ~/.docker-github-netrc
sudo chmod 400 ~/.docker-github-netrc
```

### Udev rules

To keep consistency with the devices used by Bender and to avoid that the devices won't let the docker container go up. 

```bash
source ~/uchile_system/bender_nav2/config/create_udev_rules.sh
```

### Image building

Now you will build the docker image

```bash
git clone -b feat-jazzy https://github.com/uchile-robotics/uchile_system.git
cd ~/uchile_system/bender_nav2 # For instance building the bender_nav2 container
sudo docker compose build rosaria2_container
```

## How to run

To run any piece of code you must look up the respective robot package in the organization. Independent of the packages you want to run, everything must be ran inside the docker container you just built. To run any command inside the container: `docker exec -it <container_name> /bin/bash`

- [Bender Bringup](https://github.com/uchile-robotics/bender_bringup)
- [Jaime Bringup](https://github.com/uchile-robotics/jaime_bringup)

<!-- (TODO: update jaime_bringup repository) -->
