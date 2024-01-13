# set(CMAKE_BUILD_TYPE Debug)


# find_package(Pangolin REQUIRED)

# include_directories(
#     ${Pangolin_INCLUDE_DIRS}
# )

# add_library(tools SHARED
#     Labeller.cpp
# )

# target_link_libraries(tools
#     ${OpenCV_LIBS}
#     ${EIGEN3_LIBS}
#     ${Pangolin_LIBRARIES}
# )

add_executable(label_tool label_tool.cpp)

target_link_libraries(label_tool
    # tools
    QSP-SLAM
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)
