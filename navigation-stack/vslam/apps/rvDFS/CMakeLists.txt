set( RV_DFS rv_dfs_test )

set( RV_DFS_SRC
     ./src/rvDFSTest.cpp
     )

add_definitions(-DARM_BASED)
add_definitions(-DDUMP_FILE)
add_definitions(-DRECTIFIED_INPUT)
add_definitions(-DDFS_CPP_STYLE_INTERFACE)
#add_definitions(-DPROFILING)


if (CMAKE_CROSSCOMPILING)
    include_directories (${SYSROOT_INCDIR}/opencv2)
else ()
    find_package(OpenCV REQUIRED)
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif ()

add_executable( ${RV_DFS} ${RV_DFS_SRC} )

target_link_libraries(${RV_DFS}
    rv
    fastcvopt
    dl
    pthread
    z
    jpeg
    webp
    png
    tiff
    png
)

if (CMAKE_CROSSCOMPILING)
	#target_link_libraries(${RV_DFS} ${OpenCV_LIBS})
else ()
    target_link_libraries(${RV_DFS} ${OpenCV_LIBS})
endif ()



install( TARGETS ${RV_DFS}
	RUNTIME DESTINATION /usr/bin
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
