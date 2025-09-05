# UChile System, Bender Core

## Table of contents

## Installation

### Udev rules

To ensure consistency in the devices used by any robot, and to avoid issues when starting the Docker container, install the udev rules:

```bash
source ~/uchile_robotics/uchile_system/bender/core/config/udev_scripts/create_udev_rules.sh
```

To remove these device symlinks, run:

```bash
source ~/uchile_robotics/uchile_system/bender/core/config/udev_scripts/delete_udev_rules.sh
```
