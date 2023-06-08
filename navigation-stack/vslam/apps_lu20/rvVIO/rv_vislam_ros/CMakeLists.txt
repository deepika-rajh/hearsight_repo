include(../../ros2_common.cmake)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

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
  ${ROS2_FOXY_COMMON_LIBS}
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
