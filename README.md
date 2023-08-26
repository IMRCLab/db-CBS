# db-CBS

## Preparation

The default version of OMPL 1.5.2 needs to be updated to at least 1.6.0. The following installs OMPL with the Python bindings

```
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh
chmod +x install-ompl-ubuntu.sh
./install-ompl-ubuntu.sh -p
```

## Building

Tested on Ubuntu 22.04.

```
mkdir buildRelease
cd buildRelease
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

## Running

```
cd buildRelease
python3 ../scripts/benchmark.py
```

## ROS
ROS2 workspace needs to have [crazyswarm2](https://github.com/IMRCLab/crazyswarm2). 

**Set Up**

Add a symlink of dbcbs_ros to your ROS2 workspace

```
ln <PATH-TO>/dbcbs_ros <PATH-TO>ros2_ws/src/ -s
```

**Build** 

```
colcon build --symlink-install 
```

**Usage**

Note that all configuration files are the ones used in cvmrs_ros/config. This allows to commit those files without changing the default value of the (public) crazyswarm2 repository. 

```
ros2 launch dbcbs_ros launch.py
```

and in a separate terminal

```
ros2 run dbcbs_ros multi_trajectory