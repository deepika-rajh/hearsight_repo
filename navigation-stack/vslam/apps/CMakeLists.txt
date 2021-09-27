include_directories(inc)

if (VSLAM_ENABLED)
    add_subdirectory(dvslam)
    add_subdirectory(mono_slam)
    if (ROS_ENABLED)
        add_subdirectory(dvslam_ros)
        add_subdirectory(mono_slam_ros)
    endif()
endif()

if (DFS_ENABLED)
    add_subdirectory(rvDFS)
endif()
