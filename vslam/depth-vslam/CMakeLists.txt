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
  add_definitions(-DWHEEL_SUPPORTED)
  add_definitions(-DOPENCV_ENABLED)
  add_definitions(-DIMU_SUPPORTED)

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
