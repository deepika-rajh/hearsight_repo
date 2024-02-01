cmake_minimum_required(VERSION 3.5)
project(dfs)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories (
  include
  ../vslam/prebuilt/inc/
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(../vslam/prebuilt/libs/)

add_executable(${PROJECT_NAME}
    src/dfs_ros2.cpp)

target_link_libraries(${PROJECT_NAME} 
    rv
    mv1
    mv3
    ${OpenCV_LIBS}
    pthread
    dl
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp 
    message_filters
    image_transport
    cv_bridge
    std_msgs 
    sensor_msgs
)

install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
	    DESTINATION share/${PROJECT_NAME})

ament_package()
 
