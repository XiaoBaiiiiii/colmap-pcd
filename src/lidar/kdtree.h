#ifndef COLMAP_LIDAR_KDTREE_H
#define COLMAP_LIDAR_KDTREE_H

#include <vector>
#include <QCoreApplication>
#include <pcl/kdtree/kdtree_flann.h>
#include "lidar/pt_type.h"

namespace colmap{
namespace lidar{

class Kdtree{
  public:
    Kdtree(){}
    ~Kdtree(){}
    void BuildMap(LidarPointcloudPtr& ptr);
    bool GetClosestPoint(const LidarPoint& pt, LidarPoint& closest_point);
  private:
    LidarPointcloudPtr global_pointcloud_ptr_;
    pcl::KdTreeFLANN<LidarPoint> kdtree_;
};

}
}

#endif