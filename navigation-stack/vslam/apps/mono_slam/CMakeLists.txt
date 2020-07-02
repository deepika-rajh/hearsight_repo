set( RV_VWSLAM rv_vwslam )
set( RV_VWSLAM_SRC
     Main.cpp
     ../src/VSLAMSystem.cpp
     ../src/SystemTime.cpp
     ../src/InputCamera_ov9282.cpp
     ../src/VSLAMIMU.cpp
     ../src/InputIMU.cpp
     ../src/VSLAMWheel.cpp
     ../src/VSLAMHijack.cpp
     ../src/Visualization.cpp
     )

find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

include_directories(${GLIB_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})

add_definitions(-DARM_BASED)
#add_definitions(-DOPENCV_TEST)

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable( ${RV_VWSLAM} ${RV_VWSLAM_SRC} )

target_link_libraries(${RV_VWSLAM}
	rv mv3
	${GSTREAMER_LIBRARIES} ${GST_APP_LIBRARIES}
	pthread dl
	)
target_link_libraries(${RV_VWSLAM} ${OpenCV_LIBS})

install( TARGETS ${RV_VWSLAM}
        RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
