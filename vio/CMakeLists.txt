cmake_minimum_required(VERSION 3.5)
project(vio)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rosidl_adapter REQUIRED)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED)

include_directories (
  include
  ../vslam/prebuilt/inc/
  ../vslam/include
  ${SYSROOT_INCDIR}/gstreamer-1.0
  ${SYSROOT_INCDIR}/../lib/glib-2.0/include
  ${SYSROOT_INCDIR}/glib-2.0
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(../vslam/prebuilt/libs/)

add_executable(${PROJECT_NAME}
    src/Vio.cpp
    src/VISLAMSystem.cpp
    ../vslam/src/SystemTime.cpp
    ../vslam/src/InputCamera_ov9282.cpp
    ../vslam/src/VSLAMIMU.cpp
    ../vslam/src/VSLAMWheel.cpp
    ../vslam/src/VSLAMHijack.cpp
    ../vslam/src/Visualization.cpp
    ../vslam/src/InputWheelROS.cpp
    ../vslam/src/IMURecorder.cpp
    ../vslam/src/ParseSensorParam.cpp
)

target_link_libraries(${PROJECT_NAME}
    rv
    mv1
    mv3
    sensor_client
    gstreamer-1.0
    ${OpenCV_LIBS}
    glib-2.0
    gobject-2.0
    pthread
    dl)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
    cv_bridge
    image_transport)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

ament_package()

