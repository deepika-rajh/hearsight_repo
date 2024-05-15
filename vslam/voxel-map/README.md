## Introduction

voxel-map is used to support for Voxel Map and point cloud when running vSLAM. It can generate a 2D navigation map by publishing /vm/occupany_img; it also uses the depth info and odometry info by subscribing to/vslam_odom_rawwhen the vslam is running. The odometry is from your robot.

## Prerequisite

- The RealSenseTM D455 camera is available.
- Wi-Fi is enabled with the steps in [Configure Wi-Fi](https://docs.qualcomm.com/bundle/resource/topics/80-63942-3_53398/configue-wifi_3_1_3.html).
- Generate and install slam-gmapping IPKs in [Configure slam-gmaaping](https://docs.qualcomm.com/bundle/resource/topics/80-63942-3_53398/slam-gmapping-ipk-generation-and-installation_3_1_4.html)
- Enable ssh:
```bash
adb shell "setenforce 0"
adb shell "systemctl restart sshdgenkeys.service"
```

## Build

Build voxel-map in QIRP SDK:
```bash
cd <qirp-workspace>
source setup.sh

cd sample-code/qirp_nodes/navigation_nodes/vslam/voxel-map
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

Push voxel-map to device:
Please make sure QIRP is installed on device before you push the VSLAM to device.
```bash
cd sample-code/qirp_nodes/navigation_nodes/vslam/voxel-map/install
tar czvf voxel-map.tar.gz lib share
adb push voxel-map.tar.gz /opt/
adb shell "tar -zxf /opt/voxel-map.tar.gz -C /opt/qcom/qirp-sdk/usr/"
```

## Configuration

**1. RB3 Gen2 environment configuration**
```bash
cd /opt/qcom/qirp-sdk/etc
vi car.conf
# add 2 lines
#     - car_type:0
#     - rc_enable:0

```

**2. ssh using in Ubuntu PC (need in the same wifi with device)**

```bash
ssh root@IP
```

## Run

All terminals need to execute the following commands:

```bash
export HOME=/opt
source /usr/bin/ros_setup.sh && source /opt/qcom/qirp-sdk/qirp-setup.sh
export ROS_DOMAIN_ID=xxx
```
> Note: Value range of `ROS_DOMAIN_ID`: [0, 232]

**1. Run Realsense ROS2 node    //terminal1**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/qcom/qirp-sdk/data/misc/vwslam/Configuration/vslam-node_profile.xml
ros2 launch realsense2_camera rs_launch.py enable_sync:=true align_depth.enable:=true rgb_camera.profile:=848x480x30 depth_module.profile:=848x480x30
```

**2. Run depth-vSLAM in depth_init mode   //terminal2**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/qcom/qirp-sdk/data/misc/vwslam/Configuration/vslam-node_profile.xml
cd /opt/qcom/qirp-sdk/usr/lib/depth-vslam
chmod 777 depth-vslam
./depth-vslam
```

**3. Run robot-control ROS2 node    //terminal3**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/qcom/qirp-sdk/data/misc/vwslam/Configuration/vslam-node_profile.xml
ros2 launch qti_robot_amr_ctrl qti_robot_amr_ctrl.launch.py
```

**4. Run voxel-map ROS2 node    //terminal4**

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/opt/qcom/qirp-sdk/data/misc/vwslam/Configuration/vslam-node_profile.xml
cd /opt/qcom/qirp-sdk/usr/lib/voxel-map
chmod 777 voxel-map
./voxel-map
```

**5. Run keyboard-ctrl ROS2 node    //terminal5**

```bash
ros2 run qti_robot_keyboard qti_keyboard
```

## Ubuntu PC configuration

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=xx
rviz2
```

configure Rviz2, add the following topics

- select "frame" as "/odom"
- `add "/vm/occupancy_img" by topic`
- `add "TF","Grid" by display type`
