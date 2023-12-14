cmake_policy(SET CMP0048 NEW)
project("QSP-SLAM")
cmake_minimum_required(VERSION 3.2)

# c++ version
set(CMAKE_CXX_STANDARD 17)

# 关闭所有warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Debug)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


set(OpenCV_DIR "/usr/local/opencv-3.4/share/OpenCV")

# # find_package(OpenCV 3.2 REQUIRED)
# set(OpenCV_DIR /home/lj/Downloads/Install/opencv-3.4.10/build)
# # set(Pangolin_DIR /home/lj/Documents/DSP-SLAM/Thirdparty/Pangolin/build)

find_package(OpenCV 3.4.1 REQUIRED)
find_package(Eigen3 3.4.0 REQUIRED)
find_package(Pangolin REQUIRED)

set(Pangolin_DIR "/home/lj/Documents/DSC-SLAM/Thirdparty/Pangolin/build")

set(CONDA_ENV "/home/lj/anaconda3/envs/dsp-slam2")

set(DSC_SLAM_DIR "/home/lj/Documents/DSC-SLAM")

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${DSC_SLAM_DIR}/Thirdparty/g2o
        ${DSC_SLAM_DIR}/Thirdparty/DBoW2
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)

# You need to set to your own Python paths
add_subdirectory(pybind11)
# add_subdirectory(/home/lj/Documents/DSC-SLAM/pybind11)
# include_directories(/home/lj/Documents/DSC-SLAM/pybind11)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/System_util.cc
        src/Tracking.cc
        src/Tracking_util.cc
        src/LocalMapping.cc
        src/LocalMapping_util.cc
        src/LoopClosing.cc
        src/LoopClosing_util.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/MapPoint_util.cc
        src/KeyFrame.cc
        src/KeyFrame_util.cc
        src/Map.cc
        src/Map_util.cc
        src/MapDrawer.cc
        src/MapDrawer_util.cc
        src/Optimizer.cc
        src/Optimizer_util.cc
        src/PnPsolver.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Initializer.cc
        src/Viewer.cc
        src/ObjectDetection.cc
        src/MapObject.cc
        src/ObjectDrawer.cc
        src/ObjectRenderer.cc
        )
target_link_libraries(${PROJECT_NAME}
        pybind11::embed
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${DSC_SLAM_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${DSC_SLAM_DIR}/Thirdparty/g2o/lib/libg2o.so
        )

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
# add_executable(qsp_slam qsp_slam.cc)
# target_link_libraries(qsp_slam ${PROJECT_NAME})

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
# add_executable(qsp_slam_mono qsp_slam_mono.cc)
# target_link_libraries(qsp_slam_mono ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
add_executable(qsp_slam_rgbd qsp_slam_rgbd.cc)
target_link_libraries(qsp_slam_rgbd ${PROJECT_NAME})