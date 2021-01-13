include_directories(inc)

add_subdirectory(dvslam)
add_subdirectory(mono_slam)
if (ROS_FOXY)
    add_subdirectory(dvslam_ros)
    add_subdirectory(mono_slam_ros)
endif()
