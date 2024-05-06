cmake_minimum_required(VERSION 3.8.2)
project(qrb_ros_vio
    LANGUAGES C CXX)

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

  include_directories (
    include
    ../vslam/prebuilt/inc/
    ../vslam/include
    ${OpenCV_INCLUDE_DIRS}
  )

  link_directories(../vslam/prebuilt/libs/)
  
  # find dependencies
  find_package(ament_cmake_auto REQUIRED)

  set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
  find_package(OpenCV REQUIRED)

  ament_auto_find_build_dependencies()
  ament_auto_add_executable(vio_node src/vio_node.cpp)

  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/vio_component.cpp
    src/VISLAMSystem.cpp
    src/VISLAMSystemROS2.cpp
    ../vslam/src/SystemTime.cpp
    ../vslam/src/InputOv9282ROS2.cpp
    ../vslam/src/Visualization.cpp
    ../vslam/src/ParseSensorParam.cpp
    ../vslam/src/InputIMUROS2.cpp
  )

  rclcpp_components_register_nodes(${PROJECT_NAME}
    PLUGIN "qrb_ros_vio::VioComponent"
  )

  target_link_libraries(${PROJECT_NAME}
    rv
    mv1
    mv3
    image_type_adapter
    pthread
    dl
  )

  target_link_libraries(vio_node
    ${PROJECT_NAME}
  )

  ament_auto_package(INSTALL_TO_SHARE launch)

else()
  include_directories(./include)

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
