cmake_minimum_required(VERSION 3.8)
project(sensor_service VERSION 2.0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-g)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML yaml-cpp)
pkg_search_module(SENSORSAPI REQUIRED sensors-api)
pkg_search_module(SENSORSUTILS REQUIRED sensors-utils)
pkg_search_module(SENSINGHUB REQUIRED sensing-hub)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YAML_INCLUDE_DIRS})
include_directories(${SENSORSAPI_INCLUDE_DIRS})
include_directories(${SENSORSUTILS_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}
    src/config_parser.cpp
    src/sensor_manager.cpp
    src/imu_sensor.cpp
    src/circule_buffer.cpp
    src/session_comm.cpp
    src/main.cpp
    )

target_compile_definitions(${PROJECT_NAME} PRIVATE -DSNS_SUPPORT_DIRECT_CHANNEL)

target_link_libraries(${PROJECT_NAME}
    cdsprpc
    protobuf
    ${YAML_LIBRARIES}
    ${SENSORSAPI_LIBRARIES}
    ${SENSORSUTILS_LIBRARIES}
    ${SENSINGHUB_LIBRARIES}
    )

if(BUILD_IN_YOCTO)
    target_link_libraries(${PROJECT_NAME}
        absl_log_internal_check_op
        absl_log_internal_message
    )
endif()


install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
    )

install(DIRECTORY config/
    DESTINATION /etc/
    )
