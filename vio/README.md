This is README for robotics-sample VIO.

# Introduction
VIO, which stands for Visual-Inertial Odometry, is a navigation and positioning technology used in robotics and autonomous vehicles. It combines visual information from cameras with data from an Inertial Measurement Unit (IMU) to estimate the system's motion and position.

The vio publishes and subscribes to the following topics.

- Published topics
```
/robot_odom: nav_msgs/msg/Odometry
/vslam/labeled_img: sensor_msgs/msg/Image
/vslam_odom_raw: nav_msgs/msg/Odometry
```
- Subscribed-to topics
```
/image: sensor_msgs/msg/Image
/imu: sensor_msgs/msg/Imu
/vslam_state: std_msgs/msg/String
```

# Prerequisite

- RB3 gen2 VK
- Wi-Fi is enabled with the steps in [Configure Wi-Fi](https://docs.qualcomm.com/bundle/resource/topics/80-63942-3_53398/configue-wifi_3_1_3.html).
- Setup SSH with the steps in [How to ssh](https://docs.qualcomm.com/bundle/resource/topics/80-70014-254_55940/how_to.html#how-to-ssh-)

# Compile VIO node

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

## Build VIO source

Build VIO in QIRP SDK:
```bash
cd <qirp-workspace>
source setup.sh

cd sample-code/qirp_nodes/navigation_nodes/vio
export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

colcon build --merge-install --cmake-args \
  -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
  -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
  -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu \
  -DCMAKE_STAGING_PREFIX=$(pwd)/install \
  -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
  -DBUILD_TESTING=OFF
```

Push VIO to device:
Please make sure QIRP is installed on device before you push the VIO to device.
```bash
cd sample-code/qirp_nodes/navigation_nodes/vio/install
tar czvf vio.tar.gz lib share
ssh root@[ip-addr]
(ssh) mount -o remount rw /
scp vio.tar.gz root@[ip-addr]:/home/
ssh root@[ip-addr]
(ssh) tar -zxf /home/vio.tar.gz -C /usr/
```

# Configuration

**1. ssh using in Ubuntu PC (need in the same wifi with device)**

```bash
ssh root@IP
```

## Run

**1. Run vio in #terminal1**

```bash
(ssh) export HOME=/home
(ssh) source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh
(ssh) export ROS_DOMAIN_ID=xx
(ssh) setenforce 0
(ssh) ros2 launch qrb_ros_vio vio.launch.py
```
> Note: Value range of `ROS_DOMAIN_ID`: [0, 232]

## Ubuntu PC configuration

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=xx
rviz2
```

configure Rviz2, add the following topics

- select "frame" as "/odom"
- add "vslam/labeled_img","/vslam_odom_raw" and "robot_odom" and "/vslam_state" by topic
- add "TF","Grid" by display type