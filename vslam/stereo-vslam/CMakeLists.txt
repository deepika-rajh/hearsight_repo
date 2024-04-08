cmake_minimum_required(VERSION 3.5)
project(stereo-vslam)

option(BUILD_TESTS_ONLY "Build only the tests" OFF)

if(NOT BUILD_TESTS_ONLY)
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

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(rosidl_adapter REQUIRED)
  find_package(OpenCV REQUIRED)

  include_directories (
  ../include
  ../prebuilt/inc/
  ${OpenCV_INCLUDE_DIRS}
  )

  link_directories(../prebuilt/libs/)

  add_executable(${PROJECT_NAME}
    src/StereovSLAM.cpp
    ../src/VSLAMSystem.cpp
    ../src/SystemTime.cpp
    ../src/InputStereoCameraROS2.cpp
    ../src/VSLAMWheel.cpp
    ../src/VSLAMHijack.cpp
    ../src/Visualization.cpp
    ../src/InputWheelROS.cpp
    ../src/ParseSensorParam.cpp
  )

  target_link_libraries(${PROJECT_NAME}
    rv
    mv1
    mv3
    ${OpenCV_LIBS}
    pthread
    dl)

  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    nav_msgs
    geometry_msgs
    std_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    image_transport)

  install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

  ament_package()

else()
  include_directories(../include /opt/ros/foxy/include)

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
