set( RV_DVSLAM rv_dvslam )
set( RV_DVSLAM_SRC
     Main.cpp
     ../src/VSLAMSystem.cpp
     ../src/SystemTime.cpp
     ../src/InputCamera_D435i.cpp
     ../src/VSLAMIMU.cpp
     ../src/InputIMU.cpp
     ../src/VSLAMWheel.cpp
     ../src/VSLAMHijack.cpp
     ../src/Visualization.cpp
     )

add_definitions(-DARM_BASED)
add_definitions(-DENABLE_DEPTH)
#add_definitions(-DOPENCV_TEST)

if (CMAKE_CROSSCOMPILING)
    include_directories (${SYSROOT_INCDIR}/opencv2)
else ()
    find_package(OpenCV REQUIRED)
    include_directories( ${OpenCV_INCLUDE_DIRS} )
endif ()

add_executable( ${RV_DVSLAM} ${RV_DVSLAM_SRC} )

target_link_libraries(${RV_DVSLAM}
    rv mv3
    realsense2
    pthread dl
)

if (CMAKE_CROSSCOMPILING)
    #target_link_libraries(${RV_DVSLAM} ${OpenCV_LIBS})
else ()
    target_link_libraries(${RV_DVSLAM} ${OpenCV_LIBS})
endif ()

install( TARGETS ${RV_DVSLAM}
        RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
