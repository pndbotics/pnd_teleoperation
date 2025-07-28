# pnd-retarget

## env

- ubuntu 22
- ros2 humble
- casadi (3.6.7, install from source, binary is not compatible with ros2 humble)

## dependencies

```sh
sudo apt-get update && sudo apt-get install -y \
    build-essential \
    coinor-libipopt-dev \
    gfortran \
    liblapack-dev \
    pkg-config \
    swig \
    git \
    cmake \
    python3 \
    python3-pip \
    --install-recommends

cd /tmp && \
git clone https://github.com/casadi/casadi.git casadi && \
cd casadi && \
git checkout 3.6.7 && \
mkdir build && \
cd build && \
cmake -DWITH_PYTHON=ON -DWITH_IPOPT=ON -DWITH_OPENMP=ON -DWITH_THREAD=ON .. && \
make -j$(nproc) && \
sudo make install && \
sudo ldconfig
```

## build

```sh
source /opt/ros/humble/setup.bash
colcon build
```

## run

```sh
sudo su
./run.sh [adam_sp|adam_u] # default adam_sp
```

## preview

```sh
sudo su
./preview.sh
```


## for tests

### foxglove

foxglove studio `sudo apt install ros-$ROS_DISTRO-foxglove-bridge`

```sh
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

### all in one

```sh   
source install/setup.bash
ros2 launch bringup retarget_noitom_rosbag.py
ros2 bag play rosbag2_2025_01_23-14_23_32 -l --remap /tf:=/noitom/tf
```

### noitom mocap

```sh
source install/setup.bash
ros2 launch noitom_mocap noitom_mocap.launch.py
ros2 run noitom_mocap noitom_mocap 
```

#### record

```sh
ros2 bag record <topic_name>
```

#### play

```sh
ros2 run tests_bag play_bag --ros-args -p bag_path:=rosbag2_2025_03_28-09_04_32/
```