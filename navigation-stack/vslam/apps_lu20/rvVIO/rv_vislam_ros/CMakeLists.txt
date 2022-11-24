set( RV_VISLAM_ROS rv_vio_ros )

set( RV_VISLAM_ROS_SRC
     Main.cpp
     ../src/VISLAMSystem.cpp
     ../../src/SystemTime.cpp
     ../../src/InputCamera_ov9282.cpp
     ../../src/VSLAMIMU.cpp
     ../../src/VSLAMWheel.cpp
     ../../src/VSLAMHijack.cpp
     ../../src/Visualization.cpp
     ../../src/InputWheelROS.cpp
     ../../src/IMURecorder.cpp
     ../../src/ParseSensorParam.cpp
     )


add_definitions(-DARM_BASED)
add_definitions(-DROS_BASED)
add_definitions(-DWHEEL_SUPPORTED)
add_definitions(-DOPENCV_ENABLED)
add_definitions(-DIMU_SUPPORTED)

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories (${SYSROOT_INCDIR}/../lib/glib-2.0/include)
include_directories (${SYSROOT_INCDIR}/gstreamer-1.0)
include_directories (${SYSROOT_INCDIR}/glib-2.0)
include_directories (${ROS_DIR}/include)
link_directories(${ROS_DIR}/lib)
link_directories(${ROS_DIR}/lib/aarch64-linux-gnu)

add_executable( ${RV_VISLAM_ROS} ${RV_VISLAM_ROS_SRC} )

target_link_libraries(${RV_VISLAM_ROS}
  gstreamer-1.0
  glib-2.0
  gobject-2.0
  imu_client
  rclcpp
  rcutils
  rcpputils
  yaml
  rcl
  rcl_interfaces__rosidl_generator_c
  rcl_interfaces__rosidl_typesupport_c
  rcl_interfaces__rosidl_typesupport_cpp
  rcl_yaml_param_parser
  rosgraph_msgs__rosidl_typesupport_cpp
  nav_msgs__rosidl_typesupport_cpp
  std_msgs__rosidl_typesupport_cpp
  sensor_msgs__rosidl_typesupport_cpp
  message_filters
  ament_index_cpp
  image_transport
  class_loader
  rmw
  rmw_implementation
  rosidl_typesupport_c
  rosidl_typesupport_cpp
  rosidl_runtime_c
  rcl_logging_spdlog
  builtin_interfaces__rosidl_generator_c
  libstatistics_collector
  statistics_msgs__rosidl_typesupport_cpp
  console_bridge
  cv_bridge
  tracetools
)


target_link_libraries(${RV_VISLAM_ROS}
    rv mv1 mv3
    ${OpenCV_LIBS}
    pthread
    dl
)

install( TARGETS ${RV_VISLAM_ROS}
	RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
