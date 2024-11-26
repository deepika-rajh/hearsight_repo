This is README for robotics-sample auto-mapping.

# Introduction

This sample is to build a 2D map for ground robot automatically. The robot will return back the setting starting point after completing mapping.

# Prerequisite

- The rplidar A3 is available.
- Wi-Fi is enabled with the steps in [Configure Wi-Fi](https://docs.qualcomm.com/bundle/resource/topics/80-63942-3_53398/configue-wifi_3_1_3.html).
- Setup SSH with the steps in [How to ssh](https://docs.qualcomm.com/bundle/resource/topics/80-70014-254_55940/how_to.html#how-to-ssh-)

  

# Compile auto-mapping node

## Setup QIRP cross-compile environment

```
tar -zxf qirp_sdk_<qirp_version>.tar.gz

tree qirp-sdk -L 1
qirp-sdk
├── runtime
├── sample-code
├── setup.sh
└── toolchain

cd qirp-sdk
source setup.sh
```



## Build auto-mapping source

```bash
cd  sample-code/qirp_nodes/navigation_nodes/qrb_ros_auto_mapping/auto-explore

export AMENT_PREFIX_PATH="${OECORE_NATIVE_SYSROOT}/usr:${OECORE_TARGET_SYSROOT}/usr"
export PYTHONPATH=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.10/site-packages/:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/

colcon build --cmake-args -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include  -DCMAKE_STAGING_PREFIX=$(pwd)/install -DCMAKE_PREFIX_PATH=$(pwd)/install/share -DBUILD_TESTING=OFF -DCMAKE_MAKE_PROGRAM=/usr/bin/make
```

Push auto-mapping binaries to device:
Please make sure QIRP is installed on device before you push the auto-explore to device.

```bash
cd sample-code/qirp_nodes/navigation_nodes/qrb_ros_auto_mapping/auto-explore/install/auto-explore
tar czvf auto-explore.tar.gz lib share
ssh root@[ip-addr]
(ssh) mount -o remount rw /
scp auto-explore.tar.gz root@[ip-addr]:/home/
ssh ssh root@[ip-addr]
(ssh) tar -zxf /home/auto-explore.tar.gz -C /usr/
```

# Configuration

**1. RB3 Gen2 environment configuration**
```bash
(ssh) cd /etc
(ssh) vi car.conf
# add 2 lines
#     - car_type:0
#     - rc_enable:0
```

**2. ssh using in Ubuntu PC (need in the same wifi with device)**

```bash
ssh root@IP
```



# Run auto-mapping node

## Open  5 terminal to run auto-mapping function

All terminals run in Ubuntu PC and All terminals need to execute the following commands:

```
(ssh) export HOME=/home 
(ssh) source /usr/bin/ros_setup.sh && source /etc/profile.d/qirp-setup.sh
(ssh) export ROS_DOMAIN_ID=xxx
(ssh) setenforce 0
```
> Note: Value range of `ROS_DOMAIN_ID`: [0, 232]

**Run robot-bringup node   //terminal1**

```
(ssh) ros2 launch qti_robot_amr_ctrl qti_robot_amr_ctrl.launch.py
```

**Run 2D lidar ROS node   //terminal2**

```
(ssh) ros2 launch rplidar_ros rplidar_a3_launch.py
```

**Run slam-gmapping node   //terminal3**

```
(ssh) ros2 launch slam_gmapping slam_gmapping.launch.py
```

**Run nav2_bringup node   //terminal4**

```
(ssh) source /usr/share/nav2_bringup/local_setup.bash
(ssh) ros2 launch nav2_bringup navigation_launch.py 'use_sim_time:=false'
```

**Run auto-explore  //terminal5**

```
(ssh) cd /usr/lib/auto-explore
(ssh) chmod 777 auto-explore
(ssh) ./auto-explore
```

## Ubuntu PC configuration

```
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=34
rviz2

- /plan
- /received_global_plan
- /goal_pose
```

