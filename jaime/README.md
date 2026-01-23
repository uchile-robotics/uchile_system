# UChile System, Bender

## Table of contents

- [UChile System, Bender](#uchile-system-bender)
  - [Table of contents](#table-of-contents)
  - [Installation](#installation)
  - [Container startup](#container-startup)

## Installation

For the installation please read [this readme](../README.md)
nota: a veces falla el colcon build, en este caso se debe eliminar log/, build/ e install/ de jaime_ws y jaime_core, luego realizar nuevamente colcon build

## Container startup

```bash
sudo docker compose up -d --build --remove-orphans # if you have already built the image you can skip the --build flag
```
