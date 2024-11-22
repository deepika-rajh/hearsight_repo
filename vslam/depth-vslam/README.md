## Introduction
vSLAM can be based on either monocular camera or RGBD camera. Depth-vslam is going to add depth support interface and workflow. It can generate odometry continuously using a precaptured sensor sequence as the input. The odometry is from your robot.

The depth-vslam publishes and subscribes to the following topics.

- Published topics
```
/vslam/labeled_img
/vslam_odom_raw
/robot_odom
/sensor_imu
```
- Subscribed-to topics
```
/camera/color/image_raw
/camera/aligned_depth_to_color/image_raw
/odom
```

## Prerequisite

- The RealSenseTM D455 camera is available.
- Wi-Fi is enabled with the steps in [Configure Wi-Fi](https://docs.qualcomm.com/bundle/resource/topics/80-63942-3_53398/configue-wifi_3_1_3.html).
- Generate and install slam-gmapping IPKs in [Configure slam-gmaaping](https://docs.qualcomm.com/bundle/resource/topics/80-63942-3_53398/slam-gmapping-ipk-generation-and-installation_3_1_4.html)
- Setup SSH with the steps in [How to ssh](https://docs.qualcomm.com/bundle/resource/topics/80-70014-254_55940/how_to.html#how-to-ssh-)

## Build

Build VSLAM in QIRP SDK:
```bash
cd <qirp-decompressed-workspace>
source setup.sh

cd sample-code/qirp_nodes/navigation_nodes/vslam/depth-vslam
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

Push VSLAM to device:
Please make sure QIRP is installed on device before you push the VSLAM to device.
```bash
cd sample-code/qirp_nodes/navigation_nodes/vslam/depth-vslam/install/depth-vslam
tar czvf vslam.tar.gz lib share
ssh root@[ip-addr]
(ssh) mount -o remount rw /
scp vslam.tar.gz root@[ip-addr]:/home/
ssh ssh root@[ip-addr]
(ssh) tar -zxf /home/vslam.tar.gz -C  /usr/
```

## Configuration

**1. RB3 Gen2 environment configuration**
```bash
(ssh) cd  /etc
(ssh) vi car.conf
# add 2 lines
#     - car_type:0
#     - rc_enable:0
```

**2. ssh using in Ubuntu PC (need in the same wifi with device)**

```bash
ssh root@IP
```

## Run

Note: All terminals run in Ubuntu PC.

All terminals need to execute the following commands:

```bash
(ssh) export HOME=/home
(ssh) source /usr/bin/ros_setup.sh && source /usr/share/qirp-setup.sh
(ssh) export ROS_DOMAIN_ID=xxx
(ssh) setenforce 0
```
> Note: Value range of `ROS_DOMAIN_ID`: [0, 232]

**1. Run Realsense ROS2 node    //terminal1**

```bash
(ssh) export FASTRTPS_DEFAULT_PROFILES_FILE=/data/misc/vwslam/Configuration/vslam-node_profile.xml
(ssh) ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=true rgb_camera.profile:=848x480x30 depth_module.profile:=848x480x30
```

**2. Run depth-vSLAM in depth_init mode   //terminal2**
```bash
(ssh) export FASTRTPS_DEFAULT_PROFILES_FILE=/data/misc/vwslam/Configuration/vslam-node_profile.xml
(ssh) cd /usr/lib/depth-vslam
(ssh) chmod 777 depth-vslam
(ssh) ./depth-vslam
```

**3. Run robot-control ROS2 node    //terminal3**

```bash
(ssh) export FASTRTPS_DEFAULT_PROFILES_FILE=/data/misc/vwslam/Configuration/vslam-node_profile.xml
(ssh) ros2 launch qti_robot_amr_ctrl qti_robot_amr_ctrl.launch.py
```

**4. Run keyboard-ctrl ROS2 node    //terminal4**

```bash
(ssh) ros2 run qti_robot_keyboard qti_keyboard
```

## Ubuntu PC configuration

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=xx
rviz2
```

configure Rviz2, add the following topics

- select "frame" as "/odom"
- `add "vslam/labeled_img","odom" and "robot_odom" by topic`
- `add "TF","Grid" by display type`
