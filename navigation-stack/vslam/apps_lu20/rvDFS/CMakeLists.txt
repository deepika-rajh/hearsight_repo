set( RV_DFS rv_dfs_test )

set( RV_DFS_SRC
     ./src/rvDFSTest.cpp
     ./src/dfs_test_tools.cpp
     )

add_definitions(-DARM_BASED)
add_definitions(-DDUMP_FILE)
add_definitions(-DRECTIFIED_INPUT)
add_definitions(-DDFS_CPP_STYLE_INTERFACE)
add_definitions(-DPROFILING)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(inc)

add_executable( ${RV_DFS} ${RV_DFS_SRC} )

target_link_libraries(${RV_DFS}
    rv
    mv1
    mv3
    ${OpenCV_LIBS}
    boost_filesystem
    pthread
    dl
)


install( TARGETS ${RV_DFS}
	RUNTIME DESTINATION /usr/bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
