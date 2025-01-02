# Setup

Install `ros2 Jazzy` on `Ubuntu 24.04`

## Dependencies

### Install RosDep

```Bash
apt-get install python3-rosdep
```

### Initialize DependenciesInstall RosDep

```bash
sudo rosdep init
rosdep update
```

### Install/Update Dependencies

```bash
rosdep install --from-paths src -y --ignore-src
```

## Build

```bash
colcon buld # initializes build, install, and log file ignored on github
```

## Source

```bash
source install/local_setup.sh # sources dependencie, build files, etc.
```

# Launch Scripts

## Drivetrain Launch

Activates nodes for drivetrain only

- main_drive
- main_joy

```Bash
ros2 launch launch/drive_launch.py
```

## Full Launch

Activates broker publisher, sensor nodes, drivetrain, and autonomy

- main_drive
- main_joy
- autonomous
- sensor
- 4x broker [can_ids 15-18]

```Bash
ros2 launch launch/main_launch.py
```

## LiDAR

Test script to launch LiDAR, uncomment RVIZ section to see visual model

- unitree_lidar_ros2

```Bash
ros2 launch launch/lidar_launch.py
```

**NOTE:** Won't work unless /dev/ttyUSB0 has full perms. For example, if on linux do `sudo usermod -a -G dialout $USER` to give serial ports same permissions as user.
