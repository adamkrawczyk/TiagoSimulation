# TiagoSimulation


## Installation

### Dependencies

- ROS2 Humble/Jazzy
- 16 GB of RAM
- GPU with 2 GB of RAM and DirecX 12 or Vulkan support
- Nvidia GeForce GTX 1060 or better
- 50 GB of free disk space
- O3DE 2409.2

### Building

1. Build O3DE (from source or deb package), follow orginal instructions [here](https://docs.o3de.org/docs/welcome-guide/setup/).
2. Build O3DE-Extras, instructions [here](https://github.com/o3de/o3de-extras).
3. Build TiagoSimulation
4. Run TiagoSimulation Editor (~/TiagoSimulation$ `./build/linux/bin/profile/Editor`).
5. Run simulation (click on play button in the top menu bar or press ctrl+g).

### RECOMENDED BUILD

```bash
sudo apt install -y git-lfs cmake clang libglu1-mesa-dev libxcb-xinerama0 libxcb-xinput0 libxcb-xinput-dev libxcb-xfixes0-dev libxcb-xkb-dev libxkbcommon-dev libxkbcommon-x11-dev libfontconfig1-dev libcurl4-openssl-dev libsdl2-dev zlib1g-dev mesa-common-dev libssl-dev libunwind-dev libzstd-dev ninja-build libevdev-dev ffmpeg libavcodec-dev libxcb-randr0-dev tix
sudo apt install ros-$ROS_DISTRO-ackermann-msgs ros-$ROS_DISTRO-gazebo-msgs ros-$ROS_DISTRO-control-msgs
mkdir ~/o3de_ws
cd ~/o3de_ws
git clone https://github.com/o3de/o3de.git && cd o3de && git checkout 2409.2
git lfs install
git lfs pull
./python/get_python.sh
cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_3RDPARTY_PATH=$HOME/o3de-package
cmake --build build/linux --target Editor --config profile -j <12>  # 12 is the number of cores to use for building
./scripts/o3de.sh register --this-engine
cd ~/o3de_ws && git clone https://github.com/o3de/o3de-extras && cd o3de-extras && git checkout 2409.2
cd ~/o3de_ws && git clone https://github.com/RobotecAI/o3de-rgl-gem.git && cd o3de-rgl-gem && git checkout O3DE_2409
cd ~/o3de_ws && git clone https://github.com/adamkrawczyk/TiagoRobot.git
```

Second step is to build TiagoSimulation:

```bash
mkdir ~/projects
cd ~/projects
git clone https://github.com/adamkrawczyk/TiagoSimulation.git
cd TiagoSimulation
export O3DE_HOME=~/o3de_ws/o3de
export O3DE_EXTRAS_HOME=~/o3de_ws/o3de-extras
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/ProteusRobot
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/WarehouseAssets
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/WarehouseSample
${O3DE_HOME}/scripts/o3de.sh register --template-path ${O3DE_EXTRAS_HOME}/Templates/Ros2FleetRobotTemplate
${O3DE_HOME}/scripts/o3de.sh register --template-path ${O3DE_EXTRAS_HOME}/Templates/Ros2ProjectTemplate
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/ROS2
${O3DE_HOME}/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/../o3de-rgl-gem

cmake -B build/linux -G "Ninja Multi-Config" -DLY_DISABLE_TEST_MODULES=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DLY_STRIP_DEBUG_SYMBOLS=ON -DAZ_USE_PHYSX5:BOOL=ON 
cmake --build build/linux --config profile --target TiagoSimulation Editor TiagoSimulation.Assets
./build/linux/bin/profile/Editor
```

Run simulation (click on play button in the top menu bar or press ctrl+g).

The topics should be visible in the terminal where you run the simulation (`ros2 topic list`).

To run teleop controll: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --remap /cmd_vel:=/base_footprint/cmd_vel`

## List of Components

| Nr. | Component Name | Description | Topic | Comment |
| --- | --- | --- | --- | --- |
| 1 | base_laser_link | Robotec GPU Lidar <TickBasedSource> | `scan` "10hz Best Effort Volatile" | Lidar scan |
| 2 | base_sonar_x_link | Robotec GPU lidar <TickBasedSource> | `pc` "5hz Best Effort Volatile | Point scan simulating 2D distance sensor |
| 3 | head_front_camera_rgb_frame | ROS2 Camera Sensor <TickBasedSource> | `camera_image_color` "30hz Best Effort Volatile" | RGB and depth image |
| 4 | base_imu_link | ROS2 IMU Sensor <PhysicsBasedSource> | `imu` "100hz Best Effort Volatile" | IMU data |
| 5 | base_footprint | ROS2 Odometry Sensor <PhysicsBasedSource> | `odom` "10hz Best Effort Volatile" | Odometry data |

`TickBasedSource` - data is generated in fixed time intervals based on the simulation time. Implements tick callbacks as sensor event source. Source event (ROS2::SensorEventSource) is signalled based on system ticks.
`PhysicsBasedSource` - data is generated based on the physics simulation. Implements physics callbacks as sensor event source. Source event (ROS2::SensorEventSource) is signalled based on scene. The working frequency of this event source can be changed through engine settings (physics simulation delta).
