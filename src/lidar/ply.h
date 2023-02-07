#ifndef COLMAP_LIDAR_PLY_H
#define COLMAP_LIDAR_PLY_H

#include "pt_type.h"
#include "pcd_projection.h"
#include "kdtree.h"
#include <cmath>
namespace colmap{
namespace lidar{


class PointCloudProcess{
  public:
    /**
     * @brief read and store the lidar point cloud
     * @param lidar point cloud file path
     */
    PointCloudProcess(const std::string& path):path_(path){} 
    ~PointCloudProcess(){}

    const bool Initialize(const PcdProjectionOptions& pp_options);
    /**
     * @brief Translate the lidar coordinate system to the visual coordinate system
     */
    LidarPointcloudPtr PointCloudDirectionTrans(const LidarPointcloudPtr& ptr);

    bool LoadDownsizedMap(double filter_size = 1.0);
    LidarPointcloudPtr GetDownsizedMap();

    bool SearchNearestNeiborByKdtree(const Eigen::Vector3d& point_3d, Eigen::Vector6d& l_pt);

    std::shared_ptr<PcdProj> pcd_proj_;
    std::shared_ptr<Kdtree> kdtree_ptr_;

  private:
    std::string path_;
    LidarPointcloudPtr global_pcd_ptr_;
    LidarPointcloudPtr downsized_map_ptr_;
};
} // lidar
} // colmap

#endif