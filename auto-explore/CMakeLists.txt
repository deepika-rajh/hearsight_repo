cmake_minimum_required(VERSION 3.5)

project(auto-explore
    LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Default to C99
if (NOT CMAKE_C_STANDARD)
set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()

# Common compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-function")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-format-extra-args")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-field-initializers")

if (NOT GNU)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-c99-designator")
endif ()

add_definitions(-DARM_BASED)
add_definitions(-DROS_BASED)
add_definitions(-DOPENCV_ENABLED)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories (./)
include_directories (./inc/)
include_directories (../navigation-stack/vslam/prebuilt/inc/865LU20/)
link_directories(../navigation-stack/vslam/prebuilt/libs/865LU20/)

if (CMAKE_CROSSCOMPILING)
set(ament_cmake_core_DIR "${ROS_NATIVE_DIR}/share/ament_cmake_core/cmake")
include (${ROS_NATIVE_DIR}/share/ament_cmake_core/cmake/ament_cmake_coreConfig.cmake)
include_directories (${ROS_DIR}/include)
link_directories(${SYSROOT_LIBDIR})
link_directories(${ROS_DIR}/lib)
link_directories(${ROS_DIR}/lib/aarch64-linux-gnu)
else ()
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

link_directories(/usr/lib)
link_directories(/usr/lib/aarch64-linux-gnu)
endif ()

if (CMAKE_CROSSCOMPILING)
if (ROS_VERSION STREQUAL "foxy")
set(ROS_VERSION_LIBS
    libstatistics_collector
    statistics_msgs__rosidl_typesupport_cpp
    tracetools)
else()
# TODO for error condition
endif ()
set(ROS_COMMON_LIBS
    rclcpp
    rcutils
    rcpputils
    yaml
    rcl
    rcl_interfaces__rosidl_generator_c
    rcl_interfaces__rosidl_typesupport_c
    rcl_interfaces__rosidl_typesupport_cpp
    rcl_logging_noop
    rcl_yaml_param_parser
    rosidl_runtime_c
    rosgraph_msgs__rosidl_typesupport_cpp
    message_filters
    ament_index_cpp
    class_loader
    rmw
    rmw_implementation
    rosidl_typesupport_c
    rosidl_typesupport_cpp
    std_msgs__rosidl_generator_c
    std_msgs__rosidl_typesupport_c
    std_msgs__rosidl_typesupport_cpp
    sensor_msgs__rosidl_typesupport_cpp
    console_bridge
    builtin_interfaces__rosidl_generator_c)
endif ()

if (CMAKE_CROSSCOMPILING)
set(INSTALL_PATH ${INSTALL_LIBDIR}/${PROJECT_NAME})
else ()
set(INSTALL_PATH lib/${PROJECT_NAME})
endif ()

add_executable( auto-explore
    src/test.cpp
    src/explorer.cpp)

if (CMAKE_CROSSCOMPILING)
target_link_libraries( auto-explore
    rcl_action
    rclcpp_action
    nav_msgs__rosidl_typesupport_cpp
    geometry_msgs__rosidl_typesupport_cpp
    tf2
    tf2_ros
    tf2_msgs__rosidl_typesupport_cpp
    action_msgs__rosidl_typesupport_cpp
    ${ROS_COMMON_LIBS}
    ${ROS_VERSION_LIBS}
    rv
    mv1
    mv3
    ${OpenCV_LIBS}
    pthread
    dl)
else ()
target_link_libraries( auto-explore
    ${ROS_COMMON_LIBS}
    ${ROS_VERSION_LIBS}
    rv
    mv1
    mv3
    ${OpenCV_LIBS}
    pthread
    dl)
ament_target_dependencies( auto-explore
    rclcpp
    nav_msgs
    geometry_msgs
    message_filters
    std_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs)
endif ()

install( TARGETS auto-explore
    RUNTIME DESTINATION /usr/bin/
    LIBRARY DESTINATION ${LIB_INSTALL_DIR}
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR})

ament_package()
