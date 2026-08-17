cmake_minimum_required(VERSION 3.8)
project(semantic_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)

# ---------------------------------------------------------------------------
# Optional Qualcomm QNN HTP backend.
#
# The QNN SDK is not an ament/colcon package -- it ships as part of the
# Yocto/QCS6490 cross toolchain. Point QNN_SDK_ROOT at the sysroot that
# contains include/QnnInterface.h and lib/<arch>/libQnnHtp.so, e.g.:
#
#   -DQNN_SDK_ROOT=$QNN_SDK_ROOT
#
# When QNN_SDK_ROOT is not found (e.g. building natively on a dev machine),
# the HTP backend is skipped and only the CPU (OpenCV DNN) backend is built.
# The node falls back to the CPU backend at runtime if "htp" is requested
# but was not compiled in.
# ---------------------------------------------------------------------------
option(WITH_QNN_HTP "Build the Qualcomm QNN/HTP inference backend" ON)

set(QNN_SDK_ROOT "$ENV{QNN_SDK_ROOT}" CACHE PATH "Path to the Qualcomm QNN SDK sysroot")

set(HAVE_QNN_HTP FALSE)
if(WITH_QNN_HTP AND QNN_SDK_ROOT)
  find_path(QNN_INCLUDE_DIR QnnInterface.h PATHS "${QNN_SDK_ROOT}/include" "${QNN_SDK_ROOT}/usr/include")
  if(QNN_INCLUDE_DIR)
    set(HAVE_QNN_HTP TRUE)
    message(STATUS "semantic_detection: QNN SDK found at ${QNN_INCLUDE_DIR} -- building HTP backend")
  else()
    message(WARNING "semantic_detection: WITH_QNN_HTP is ON but QnnInterface.h was not found under QNN_SDK_ROOT=${QNN_SDK_ROOT}. Skipping HTP backend.")
  endif()
endif()

set(SEMANTIC_DETECTION_SOURCES
  src/detector/YOLODetector.cpp
  src/backends/CpuBackend.cpp
  src/utils/ImageUtils.cpp
  src/utils/NMS.cpp
  src/utils/YamlParser.cpp
)

if(HAVE_QNN_HTP)
  list(APPEND SEMANTIC_DETECTION_SOURCES src/backends/QnnHtpBackend.cpp)
endif()

add_library(yolo_detector ${SEMANTIC_DETECTION_SOURCES})
target_include_directories(yolo_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(yolo_detector PUBLIC ${OpenCV_LIBS} yaml-cpp)

if(HAVE_QNN_HTP)
  # PUBLIC: YoloNode.cpp also includes QnnHtpBackend.hpp directly, so the
  # include path must propagate to consumers of this library, not just its
  # own translation units.
  target_include_directories(yolo_detector PUBLIC ${QNN_INCLUDE_DIR})
  target_compile_definitions(yolo_detector PUBLIC SEMANTIC_DETECTION_WITH_QNN_HTP)
  # libQnnHtp.so etc. are loaded at runtime with dlopen() (see QnnHtpBackend.cpp),
  # so only libdl is needed at link time, not the QNN shared libraries themselves.
  target_link_libraries(yolo_detector PUBLIC ${CMAKE_DL_LIBS})
endif()

add_executable(yolo_node src/main.cpp src/ros/YoloNode.cpp)
target_include_directories(yolo_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(yolo_node yolo_detector)
ament_target_dependencies(yolo_node
  rclcpp
  sensor_msgs
  std_msgs
  vision_msgs
  cv_bridge
  image_transport
)

install(TARGETS yolo_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS yolo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(yolo_detector)
ament_package()
