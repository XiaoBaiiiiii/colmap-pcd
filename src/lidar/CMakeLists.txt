set(FOLDER_NAME "lidar")

COLMAP_ADD_SOURCES(
    lidar_point.h lidar_point.cc
    pt_type.h
    pcd_projection.h pcd_projection.cc
    kdtree.h kdtree.cc
    ply.h ply.cc
)