# pnd-teleoperation

## Supported Robot Models

| Robot Model | Noitom | VR   | Description |
| ----------- | ------ | ---- | ----------- |
| Adam SP     | ✅      | TODO |             |
| Adam Pro    | ✅      | TODO |             |
| Adam U      | ✅      | ✅    |             |

## env

- ubuntu 22
- ros2 humble
- casadi
- uv

## dependencies

```sh
sudo apt-get update && sudo apt-get install -y \
    build-essential \
    gfortran \
    liblapack-dev \
    pkg-config \
    swig \
    git \
    cmake \
    python3 \
    python3-pip \
    git-lfs \
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

# Clone repo

```sh
git clone https://github.com/pndbotics/pnd_teleoperation.git
cd pnd_teleoperation
```

## install uv

```sh
pip install uv --user
uv sync
```

## build

```sh
./build.sh
```

## run

spteleop cli: new version command line tool, supports auto-completion and tab-completable(old version command line tool has been deprecated and will be removed in several versions)

```sh
source setup_cli.bash
spteleop teleop [adam_type] [mocap_driver] [algorithm]
# example:
spteleop
spteleop teleop adam_u webvr mink
spteleop teleop adam_pro noitom pinocchio
spteleop launch pinocchio-adam_u-webvr # launch a bringup launch file
```

## preview

```sh
sudo ./preview.sh
```

## use docker
```bash
cd docker
# update user id
./update_env.sh
docker compose up --build
```

open another terminal and connect to the container
```bash
docker exec -it pnd_retarget_ros bash
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

## FAQ

### uv sync slow

```sh
echo 'export UV_DEFAULT_INDEX="https://pypi.tuna.tsinghua.edu.cn/simple"'>> ~/.bashrc
# echo 'export UV_DEFAULT_INDEX="https://mirrors.aliyun.com/pypi/simple/"' >> ~/.bashrc
source ~/.bashrc
```

## Credits

- [telegrip](https://github.com/DipFlip/telegrip)
- [XLeRobot](https://github.com/Vector-Wangel/XLeRobot)

## Old startup script

```sh
sudo su
./run.sh [adam_type] [mocap_driver] [algorithm] # adam_type: adam_sp/adam_u/adam_pro; mocap_driver: noitom/vr; algorithm: pinocchio/mink;
# example:
./run.sh adam_u vr mink
./run.sh adam_pro noitom pinocchio
```
