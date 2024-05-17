cmake_minimum_required(VERSION 3.5)
project(auto-explore)

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
  find_package(OpenCV REQUIRED)

  include_directories (
    include
    ../vslam/prebuilt/inc/
    ${OpenCV_INCLUDE_DIRS}
  )

  link_directories(../vslam/prebuilt/libs/)

  add_executable(${PROJECT_NAME}
    src/ae_test.cpp
    src/explorer.cpp)

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
    tf2_geometry_msgs)

  install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

  ament_package()

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
