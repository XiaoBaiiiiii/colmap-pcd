#include "lidar/kdtree.h"

namespace colmap{
namespace lidar{
    void Kdtree::BuildMap(LidarPointcloudPtr& ptr){
        global_pointcloud_ptr_ = ptr;
        kdtree_.setInputCloud(global_pointcloud_ptr_);
    }

    bool Kdtree::GetClosestPoint(const LidarPoint& pt, LidarPoint& closest_point){
        int k = 1;
        std::vector<int> pointIdxNKNSearch(k);
        std::vector<float> pointNKNSquaredDistance(k);
        if (kdtree_.nearestKSearch(pt,k,pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            LidarPoint nearest_pt = global_pointcloud_ptr_->points[pointIdxNKNSearch[0]];
            closest_point = nearest_pt;
            return true;
        } else {
            return false;
        }
    }

}
}