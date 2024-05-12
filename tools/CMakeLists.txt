# # set(CMAKE_BUILD_TYPE Debug)
# add_executable(tool_transformTrajectory
#         ./transformTrajectory.cpp
# )
# target_link_libraries(tool_transformTrajectory
#     EllipsoidSLAM
# )       


# # add_executable(AlignTimestamp
# #         ./AlignTimestamp.cpp
# # )
# # target_link_libraries(AlignTimestamp
# #         EllipsoidSLAM
# # )

# # add_executable(evo_full_ellipsoids
# #         ./evo_full_ellipsoids.cpp
# # )
# # target_link_libraries(evo_full_ellipsoids
# #         EllipsoidSLAM
# # )       

# # add_executable(evo_ellipsoids_align
# #         ./evo_ellipsoids_align.cpp
# # )
# # target_link_libraries(evo_ellipsoids_align
# #         EllipsoidSLAM
# # )       

# add_executable(evo_ellipsoids_axisAligned
#     ./evo_ellipsoids_axisAligned.cpp
# )
# target_link_libraries(evo_ellipsoids_axisAligned
#     EllipsoidSLAM
#     func
# )

# add_executable(evo_tool
#     ./evo_tool.cpp
# )
# target_link_libraries(evo_tool
#     evo
# )

# include_directories(/usr/include/python3.8)

add_executable(evo_tool_qsp
    ./evo_tool_qsp.cpp
)
target_link_libraries(evo_tool_qsp
    evo
)

# add_executable(test_H
#         ./Hungarian/test_H.cpp
# )

# add_executable(basic_quaternion
#     ./calculation/basic_quaternion.cpp
# )       