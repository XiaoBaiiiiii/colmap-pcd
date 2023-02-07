#ifndef COLMAP_LIDAR_PT_TYPE_H
#define COLMAP_LIDAR_PT_TYPE_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>

namespace lidarpt 
{
  struct EIGEN_ALIGN16 Point 
  {
    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace lidarpt 

POINT_CLOUD_REGISTER_POINT_STRUCT(lidarpt::Point,
                                (float, x, x)
                                (float, y, y)
                                (float, z, z)
                                (float, normal_x, normal_x)
                                (float, normal_y, normal_y)
                                (float, normal_z, normal_z)
                                )

namespace colmap{
namespace lidar{

typedef lidarpt::Point LidarPoint;
typedef pcl::PointCloud<LidarPoint> LidarPointcloud;
typedef pcl::PointCloud<LidarPoint>::Ptr LidarPointcloudPtr;
typedef std::vector<LidarPoint,Eigen::aligned_allocator<LidarPoint>> LidarPointVector;

typedef pcl::PointXYZINormal PclPoint;
typedef pcl::PointCloud<pcl::PointXYZINormal> PclPointcloud;
typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr PclPointcloudPtr;
typedef std::vector<PclPoint,Eigen::aligned_allocator<PclPoint>> PclPointVector;

}
}

#endif