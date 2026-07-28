cmake_minimum_required(VERSION 3.5)
project(depth-vslam)

option(BUILD_TESTING "Build only the tests" OFF)

if(NOT BUILD_TESTING)
  # Default to C99
  if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
  endif()

  # Default to C++14
  if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  add_definitions(-DARM_BASED)
  add_definitions(-DROS_BASED)
  # WHEEL_SUPPORTED enables wheel odometry: the /odom subscription in
  # DepthvSLAM.cpp and the VSLAMWheel processing thread in VSLAMSystem. This rig
  # has no wheels/odom, so it is disabled to avoid subscribing to /odom.
  # To re-enable: uncomment the line below, set wheelEnabled 1 +
  #   alwaysDoMappingOnRobotOdom 1 + VSLAMIniMode DEPTHW_INIT in rgbdWSlam.cfg,
  #   and restore the WEF.Tvb/WEF.Rvb extrinsics in vslam.cfg.
  # add_definitions(-DWHEEL_SUPPORTED)
  add_definitions(-DOPENCV_ENABLED)
  # IMU_SUPPORTED enables the on-board IMU path (VSLAMIMU -> QTI SensorClient ->
  # icm42688). That thread is started unconditionally by VSLAMSystem, so leaving
  # this on makes depth-vslam always open the board's sensor service regardless of
  # config. This rig has no on-board IMU (only the RealSense D455's), so it stays
  # OFF. When IMU_SUPPORTED is off, VSLAMSystem exposes a static addIMU() that
  # InputIMUROS2 (see src/DepthvSLAM.cpp) feeds from the D455's /camera/imu topic
  # instead. To re-enable the board IMU later: uncomment the line below, set
  # IMUEnabled 1 in Configuration/rgbdWSlam.cfg, and drop the RealSense wiring in
  # DepthvSLAM.cpp.
  # add_definitions(-DIMU_SUPPORTED)

  # find dependencies
  find_package(ament_cmake_auto REQUIRED)

  set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
  find_package(OpenCV REQUIRED)

  ament_auto_find_build_dependencies()

  include_directories (
    ../include
    ../prebuilt/inc/
    ${OpenCV_INCLUDE_DIRS}
  )

  link_directories(../prebuilt/libs/)

  ament_auto_add_executable(${PROJECT_NAME}
    src/DepthvSLAM.cpp
    ../src/VSLAMSystem.cpp
    ../src/SystemTime.cpp
    ../src/InputRGBDCameraROS2.cpp
    ../src/InputIMUROS2.cpp
    ../src/VSLAMIMU.cpp
    ../src/VSLAMWheel.cpp
    ../src/VSLAMHijack.cpp
    ../src/Visualization.cpp
    ../src/InputWheelROS.cpp
    ../src/IMURecorder.cpp
    ../src/ParseSensorParam.cpp
  )

  target_link_libraries(${PROJECT_NAME}
    rv
    mv1
    mv3
    ${OpenCV_LIBS}
    sensor_client
    pthread
    dl)

  ament_auto_package(INSTALL_TO_SHARE launch)

else()
  include_directories(../include)

  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(ament_cmake_auto REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  set(TESTFILES
      test/src/main.cpp
      test/src/test.cpp
      )

  ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
  ament_target_dependencies(${PROJECT_NAME}_test rclcpp nav_msgs sensor_msgs)

endif()
