
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
add_definitions(-D__LINUX__)

if (CMAKE_CROSSCOMPILING)
    #include_directories (${SYSROOT_INCDIR}/opencv2)
    set(OpenCV_DIR "/usr/include/opencv4/build")
    find_package(OpenCV REQUIRED)
else ()
    find_package(OpenCV REQUIRED)
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif ()

#link_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../../prebuilt/libs/2210LE/lib/)
add_executable( ${RV_DFS} ${RV_DFS_SRC} )

target_link_libraries(${RV_DFS}
    rv
    mv3
    fastcvopt
    dl
    pthread
    z
    jpeg
    webp
    png
    tiff
    opencv_imgproc
    OpenCL
)

target_link_libraries(${RV_DFS} ${OpenCV_LIBS})


install( TARGETS ${RV_DFS}
	RUNTIME DESTINATION /usr/bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
