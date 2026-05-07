cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
    message_filters
    )

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS})

set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "ONNX Runtime installation root")
find_path(ONNXRUNTIME_INCLUDE_DIR
    NAMES onnxruntime_cxx_api.h
    PATHS "${ONNXRUNTIME_ROOT}/include"
    NO_DEFAULT_PATH
)
find_library(ONNXRUNTIME_LIBRARY
    NAMES onnxruntime
    PATHS "${ONNXRUNTIME_ROOT}/lib"
    NO_DEFAULT_PATH
)

if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
    add_executable(zero_dce_onnx_enhancer
        src/zero_dce_onnx_enhancer_node.cpp
        )
    target_include_directories(zero_dce_onnx_enhancer PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
    target_link_libraries(zero_dce_onnx_enhancer ${catkin_LIBRARIES} ${OpenCV_LIBS} ${ONNXRUNTIME_LIBRARY})
else()
    message(WARNING "ONNX Runtime not found under ${ONNXRUNTIME_ROOT}; zero_dce_onnx_enhancer will not be built.")
endif()

catkin_install_python(PROGRAMS
    scripts/zero_dce_enhancer_node.py
    scripts/export_zero_dce_plus_onnx.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
