set( RV_VIO rv_vio )

set( RV_VIO_SRC
     Main.cpp
     ../../src/SystemTime.cpp
     ../../src/VSLAMIMU.cpp
     ../src/VISLAMSystem.cpp
     ../../src/Visualization.cpp
     ../../src/IMURecorder.cpp
     ../../src/ParseSensorParam.cpp
     )
 
set( RV_VIO_SIM_SRC
     ../../simulation/src/VirtualSensorDevice.cpp
     ../../simulation/src/VirtualHijack.cpp
     ../../simulation/src/VirtualIMU.cpp
     ../../simulation/src/mvReader.cpp
     ../../simulation/src/PoseReader.cpp
     ../../simulation/src/TUMRGBDReader.cpp
     ../../simulation/src/EurocReader.cpp
     ../../simulation/src/OpenLORISReader.cpp
     ../../simulation/src/OpenLORISReader_D435i.cpp
     ../../simulation/src/OpenLORISReader_T265.cpp
     ../../simulation/src/SlamDataReader.cpp
     ../../simulation/src/WheelReader.cpp
     ../../simulation/src/HijackReader.cpp
     ../../simulation/src/SystemCallbackReader.cpp
     )

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-DIMU_SUPPORTED)
add_definitions(-DOPENCV_ENABLED)
add_definitions(-DSIMULATION)

add_executable( ${RV_VIO} ${RV_VIO_SRC} ${RV_VIO_SIM_SRC})

target_link_libraries(${RV_VIO}
    rv mv1 mv3
    ${OpenCV_LIBS}
    pthread
    dl
)

install( TARGETS ${RV_VIO}
        RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
