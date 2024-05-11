

# 使用真值轨迹生成地图
add_executable(build_map_by_gt
    test/build_map_by_gt.cpp
)

target_link_libraries(build_map_by_gt
    tum_rgbd
    boost_system
    EllipsoidSLAM
    ${PCL_LIBRARIES}
)