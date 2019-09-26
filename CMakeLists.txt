cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(PCL REQUIRED)

add_definitions(-std=c++11)
include_directories(include ${PCL_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
# Ply reader
cs_add_library(ply_reader
    src/ply_reader.cpp
)
target_link_libraries(ply_reader
    ${CATKIN_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Mesh reconstructor
cs_add_library(mesh_reconstructor 
    src/mesh_reconstructor.cpp
)
target_link_libraries(mesh_reconstructor 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Pointcloud grid filter
cs_add_library(pointcloud_grid_filter 
    src/pointcloud_grid_filter.cpp
)
target_link_libraries(pointcloud_grid_filter 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

# Pointcloud frames transform
cs_add_library(pointcloud_transform_frames
    src/pointcloud_transform_frames.cpp
)
target_link_libraries(pointcloud_transform_frames 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)

############
# BINARIES #
############
# Ply reader
cs_add_executable(ply_reader_node
    src/ply_reader_node.cpp
)
target_link_libraries(ply_reader_node
    ply_reader
)

# Pointcloud repeater
cs_add_executable(pointcloud_repeater_node
    src/pointcloud_repeater_node.cpp
)
target_link_libraries(pointcloud_repeater_node
    ${CATKIN_LIBRARIES}
)

# Mesh reconstructor
cs_add_executable(mesh_reconstructor_node 
    src/mesh_reconstructor_node.cpp
)
target_link_libraries(mesh_reconstructor_node 
    mesh_reconstructor
)

# Pointcloud grid filter
cs_add_executable(pointcloud_grid_filter_node 
    src/pointcloud_grid_filter_node.cpp
)
target_link_libraries(pointcloud_grid_filter_node 
    pointcloud_grid_filter
)

# Pointcloud frames transform
cs_add_executable(pointcloud_transform_frames_node 
    src/pointcloud_transform_frames_node.cpp
)
target_link_libraries(pointcloud_transform_frames_node 
    pointcloud_transform_frames
)

##########
# EXPORT #
##########
cs_install()
cs_export()

