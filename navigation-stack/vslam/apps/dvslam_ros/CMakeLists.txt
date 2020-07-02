set( RV_DVSLAM_ROS rv_dvslam_ros )
set( RV_DVSLAM_ROS_SRC
     Main.cpp
     ../src/VSLAMSystem.cpp
     ../src/SystemTime.cpp
     ../src/InputCamera_D435i.cpp
     ../src/VSLAMIMU.cpp
     ../src/InputIMU.cpp
     ../src/VSLAMWheel.cpp
     ../src/VSLAMHijack.cpp
     ../src/Visualization.cpp
     )

add_definitions(-DARM_BASED)
add_definitions(-DENABLE_DEPTH)
add_definitions(-DROS_BASED)
include_directories(/opt/ros/dashing/include)
link_directories(/opt/ros/dashing/lib)

find_package(OpenCV REQUIRED)

add_executable( ${RV_DVSLAM_ROS} ${RV_DVSLAM_ROS_SRC} )

target_link_libraries(${RV_DVSLAM_ROS}
  rv mv3
  pthread dl
  rclcpp
  rcutils
  yaml
  rcl
  rcl_logging_noop
  rcl_interfaces__rosidl_generator_c
  rcl_interfaces__rosidl_typesupport_c
  rcl_interfaces__rosidl_typesupport_cpp
  rcl_yaml_param_parser
  rosgraph_msgs__rosidl_typesupport_cpp
  nav_msgs__rosidl_typesupport_cpp
  message_filters
  ament_index_cpp
  image_transport
  class_loader
  rmw
  rmw_implementation
  rosidl_generator_c
  rosidl_typesupport_c
  rosidl_typesupport_cpp
  console_bridge
  sensor_msgs__rosidl_typesupport_cpp
  builtin_interfaces__rosidl_generator_c
  cv_bridge
  ${OpenCV_LIBS}
  )

install( TARGETS ${RV_DVSLAM_ROS}
        RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
