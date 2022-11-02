set( RV_VWSLAM_SIM rv_vwslam_sim )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../simulation/inc)

set( RV_VWSLAM_SRC
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/SystemTime.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/VSLAMIMU.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/VSLAMWheel.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/VSLAMHijack.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/VSLAMSystem.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/Visualization.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/ParseSensorParam.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../src/IMURecorder.cpp
     )
 
set( RV_VWSLAM_SIM_SRC
     ${RV_VWSLAM_SRC}
     MainSim.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/VirtualSensorDevice.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/VirtualIMU.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/VirtualHijack.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/mvReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/SlamDataReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/WheelReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/HijackReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/SystemCallbackReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/PoseReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/OpenLORISReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/OpenLORISReader_T265.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/OpenLORISReader_D435i.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/EurocReader.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/../simulation/src/TUMRGBDReader.cpp
     )

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-DIMU_SUPPORTED)
add_definitions(-DWHEEL_SUPPORTED)
add_definitions(-DSIMULATION)

add_executable( ${RV_VWSLAM_SIM} ${RV_VWSLAM_SIM_SRC} )

target_link_libraries(${RV_VWSLAM_SIM}
      rv mv3
      ${OpenCV_LIBS}
      pthread
      dl
)

install( TARGETS ${RV_VWSLAM_SIM}
        RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
