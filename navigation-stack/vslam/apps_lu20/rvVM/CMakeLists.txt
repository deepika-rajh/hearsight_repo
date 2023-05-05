include_directories(inc)

add_subdirectory(rv_vm_sim)
if (ROS_ENABLED)
    add_subdirectory(rv_vm_ros2camera)
endif()
