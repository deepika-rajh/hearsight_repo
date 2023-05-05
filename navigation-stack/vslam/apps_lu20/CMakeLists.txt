include_directories(inc)
include_directories(simulation/inc)

add_subdirectory(rvVM)
add_subdirectory(vwslam)
add_subdirectory(rvDFS)
add_subdirectory(rvVIO)
if (ROS_ENABLED)
    add_subdirectory(dvslam_ros2camera)
    add_subdirectory(monoslam_ros2camera)
endif()

