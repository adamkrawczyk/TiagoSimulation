# TiagoSimulation


## Installation

### Dependencies

- ROS2 Humble
- 16 GB of RAM
- GPU with 2 GB of RAM and DirecX 12 or Vulkan support
- Nvidia GeForce GTX 1060 or better
- 50 GB of free disk space

### Building

1. Build O3DE (from source or deb package), follow orginal instructions [here](https://docs.o3de.org/docs/welcome-guide/setup/).
2. Build O3DE-Extras, instructions [here](https://github.com/o3de/o3de-extras).
3. Build TiagoSimulation
4. Run TiagoSimulation Editor (~/TiagoSimulation$ `./build/linux/bin/profile/Editor`).
5. Run simulation (click on play button in the top menu bar or press ctrl+g).

### RECOMENDED BUILD

```bash
sudo apt install -y git-lfs cmake clang libglu1-mesa-dev libxcb-xinerama0 libxcb-xinput0 libxcb-xinput-dev libxcb-xfixes0-dev libxcb-xkb-dev libxkbcommon-dev libxkbcommon-x11-dev libfontconfig1-dev libcurl4-openssl-dev libsdl2-dev zlib1g-dev mesa-common-dev libssl-dev libunwind-dev libzstd-dev ninja-build
mkdir ~/o3de_ws
cd ~/o3de_ws
git clone https://github.com/o3de/o3de.git
$ OPTIONAL: git checkout 4fb21b8664a54ebcfd6d3d7f036c6df7bfa7b089 # commit hash at which it was tested
git lfs install
git lfs pull
./python/get_python.sh
cmake -B build/linux -S . -G "Ninja Multi-Config" -DLY_3RDPARTY_PATH=$HOME/o3de-package
cmake --build build/linux --target Editor --config profile -j <12>  # 12 is the number of cores to use for building
./scripts/o3de.sh register --this-engine
cd ~/o3de_ws && git clone https://github.com/o3de/o3de-extras
```

Second step is to build TiagoSimulation:

```bash
mkdir ~/projects
cd ~/projects
git clone https://github.com/adamkrawczyk/TiagoSimulation.git
cd TiagoSimulation
./o3de_ws/o3de/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/ProteusRobot
./o3de_ws/o3de/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/RosRobotSample
./o3de_ws/o3de/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/WarehouseAssets
./o3de_ws/o3de/scripts/o3de.sh register --gem-path ${O3DE_EXTRAS_HOME}/Gems/WarehouseSample
./o3de_ws/o3de/scripts/o3de.sh register --template-path ${O3DE_EXTRAS_HOME}/Templates/Ros2FleetRobotTemplate
./o3de_ws/o3de/scripts/o3de.sh register --template-path ${O3DE_EXTRAS_HOME}/Templates/Ros2ProjectTemplate
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

`TickBasedSource` - data is generated in fixed time intervals based on the simulation time. Implements tick callbacks as sensor event source. Source event (ROS2::SensorEventSource) is signalled based on system ticks.
`PhysicsBasedSource` - data is generated based on the physics simulation. Implements physics callbacks as sensor event source. Source event (ROS2::SensorEventSource) is signalled based on scene. The working frequency of this event source can be changed through engine settings (physics simulation delta).
