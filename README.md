# ros_ora26

The ORA software for 2026.

## Environment setup

### Install ROS 2 Jazzy

Install instructions can be found [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Make sure to select the full desktop install (`ros-jazzy-desktop`) to ensure that we have access to GUI tools, such as RViz. Also install the dev tools (`ros-dev-tools`) so we can use `rosdep` and `colcon`. After installation, you can run this command to automatically source your ROS 2 install with every new terminal session (if you are using Bash).

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### Clone the repo
```bash
mkdir ~/ros2_ws && cd ~/ros2_ws
git clone --recurse-submodules https://github.com/oaklandrobotics/ros_ora26 src
```

> [!TIP]
> If you previously cloned the repo without the  `--recurse-submodules` flag, you can clone the submodules later by running this from the `src` directory:
> ```bash
> git submodule update --init --recursive
> ```

### Install ROS 2 dependencies

`rosdep` is a utility for installing and satisfying dependencies for ROS 2 packages. Read more about it [here](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html). 

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

> [!NOTE]
> If it throws an error about not being initialized, run these commands:
> ```bash
> sudo rosdep init
> rosdep update
> ```

### Build and source the workspace

`colcon` is a utility for compiling and building the workspace. 

```bash
# Build the workspace
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
# Source the workspace
. install/setup.bash
```

### Run (or launch) something!

You should now be able to run nodes with `ros2 run` or start launch files with `ros2 launch`.
For example, you can bring up the robot with the main launch file:

```bash
ros2 launch ora_launch robot.launch.py use_sim_time:=true use_semantic_segmentation:=false
```
*For more information on 'use_sim_time' and 'use_semantic_segmentation' launch arguments, see [Configuration Notes](#configuration-notes)*

## How to contribute

TODO

### Fork the repo (or make a branch)

### Write some code!
This can be done in your IDE of choice, such as VS Code.

### Commit and push the code

### Open a pull request and have it reviewed

### Merge it

## Configuration Notes

### Overview
This system has a defualt configuration to favor the real robot over the simulation. This means that features like semantic segmentation is enabled by defualt while simulation time is disabled by default.

To enable simulation time (Required to run the simulation on your PC), use:
```bash
ros2 launch ora_launch robot.launch.py use_sim_time:=true
```

*Semantic segmentation requires Nvidia CUDA, If you PC does not support this, disable semantic segmentation. For more information on using the semantic segmentation feature, see the [this quide.](ora_perception/semantic_segmentation/README.md)*

To disable semantic segmentation, use:
```bash
ros2 launch ora_launch robot.launch.py use_semantic_segmentation:=false
```

### Make Configuration Persistant
Rather than typing these parameters to every launch command, you can add them to your environment instead.

To disable semantic segmentation:
```bash
echo "export USE_SEMANTIC_SEGMENTATION=false" >> ~/.bashrc
```

To enable simulation time:
```bash
echo "export USE_SIM_TIME=true" >> ~/.bashrc
```

Any time an environment file is modified you must source it again or restart your shell:
```bash
source ~/.bashrc
```