include_directories(inc)

add_subdirectory(rv_vislam)
if (ROS_ENABLED)
    add_subdirectory(rv_vislam_ros)
endif()

