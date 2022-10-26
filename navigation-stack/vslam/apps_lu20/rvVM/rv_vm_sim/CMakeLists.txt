set( RV_VM_SIM rv_vm_sim )

set( RV_VM_SRC
     ../../src/SystemTime.cpp
     ../../src/VSLAMHijack.cpp
     ../src/VMSystem.cpp
     ../../src/IMURecorder.cpp
     ../../src/ParseSensorParam.cpp
     )

set( RV_VM_SIM_SRC
     ${RV_VM_SRC}
     MainSim.cpp
     ../../simulation/src/VirtualSensorDevice.cpp
     ../../simulation/src/VirtualIMU.cpp
     ../../simulation/src/VirtualHijack.cpp
     ../../simulation/src/mvReader.cpp
     ../../simulation/src/SlamDataReader.cpp
     ../../simulation/src/WheelReader.cpp
     ../../simulation/src/HijackReader.cpp
     ../../simulation/src/SystemCallbackReader.cpp
     ../../simulation/src/PoseReader.cpp
     ../../simulation/src/TUMRGBDReader.cpp
     ../../simulation/src/EurocReader.cpp
     ../../simulation/src/OpenLORISReader.cpp
     ../../simulation/src/OpenLORISReader_D435i.cpp     
     ../../simulation/src/OpenLORISReader_T265.cpp
     )

set(OpenCV_DIR "/usr/lib/aarch64-linux-gnu/cmake/opencv4/")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-DIMU_SUPPORTED)
add_definitions(-DWHEEL_SUPPORTED)
add_definitions(-DOPENCV_ENABLED)
add_definitions(-DHIJACK_SUPPORTED)

add_executable( ${RV_VM_SIM} ${RV_VM_SIM_SRC} )

target_link_libraries(${RV_VM_SIM}
    rv mv3
    ${OpenCV_LIBS}
    pthread
    dl
)

install( TARGETS ${RV_VM_SIM}
        RUNTIME DESTINATION /usr/bin/
        LIBRARY DESTINATION ${LIB_INSTALL_DIR}
        ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
