set( RV_VWSLAM_ROS rv_vwslam_ros )
set( RV_VWSLAM_ROS_SRC
     Main.cpp
     ../src/VSLAMSystem.cpp
     ../src/SystemTime.cpp
     ../src/InputCamera_ov9282.cpp
     ../src/VSLAMIMU.cpp
     ../src/InputIMU.cpp
     ../src/VSLAMWheel.cpp
     ../src/VSLAMHijack.cpp
     ../src/Visualization.cpp
     ../src/InputWheelROS.cpp
     )

add_definitions(-DARM_BASED)
add_definitions(-DROS_BASED)
add_definitions(-DWHEEL_SUPPORTED)
add_definitions(-DOPENCV_ENABLED)

if (CMAKE_CROSSCOMPILING)
    include_directories (${DASHING_DIR}/include)
    link_directories(${DASHING_DIR}/lib)
else ()
    include_directories(/opt/ros/eloquent/include)
    link_directories(/opt/ros/eloquent/lib)

    find_package(OpenCV REQUIRED)
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif ()

find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

include_directories(${GLIB_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

add_executable( ${RV_VWSLAM_ROS} ${RV_VWSLAM_ROS_SRC} )

target_link_libraries(${RV_VWSLAM_ROS}
  rv mv3
  ${GSTREAMER_LIBRARIES} ${GST_APP_LIBRARIES}
  pthread dl
  rclcpp
  rcutils
  yaml
  rcl
  rcl_interfaces__rosidl_generator_c
  rcl_interfaces__rosidl_typesupport_c
  rcl_interfaces__rosidl_typesupport_cpp
  rcl_yaml_param_parser
  rosgraph_msgs__rosidl_typesupport_cpp
  nav_msgs__rosidl_typesupport_cpp
  std_msgs__rosidl_typesupport_cpp
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
  )

if (CMAKE_CROSSCOMPILING)
    target_link_libraries(${RV_VWSLAM_ROS}
    rcl_logging_noop
    )
else ()
    target_link_libraries(${RV_VWSLAM_ROS}
        tracetools
        ${OpenCV_LIBS}
    )
endif()

install( TARGETS ${RV_VWSLAM_ROS}
        RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
