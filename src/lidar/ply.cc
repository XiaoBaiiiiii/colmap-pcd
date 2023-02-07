#include "lidar/ply.h"
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

namespace colmap{
namespace lidar{

const bool PointCloudProcess::Initialize(const PcdProjectionOptions& pp_options){
    pcd_proj_ = std::make_shared<PcdProj>(pp_options);
    kdtree_ptr_ = std::make_shared<Kdtree>();
    LidarPointcloudPtr ptr(new LidarPointcloud);

    if (pcl::io::loadPLYFile<LidarPoint>(path_,*ptr) == -1) {
        std::string error = "Couldn't read file " + path_;
        return false;
    }

    std::cout << "Read "
		<< ptr->width * ptr->height
		<< " points from "
        << path_ << std::endl;
    std::cout << std::endl;
    
    global_pcd_ptr_ = PointCloudDirectionTrans(ptr);
    // Cut point cloud to nodes
    pcd_proj_->BuildSubMap(global_pcd_ptr_);
    kdtree_ptr_->BuildMap(global_pcd_ptr_);

    return true;
}

LidarPointcloudPtr PointCloudProcess::PointCloudDirectionTrans(const LidarPointcloudPtr& ptr){
    int pt_num = ptr->points.size();
    LidarPointcloudPtr new_ptr(new LidarPointcloud());
    new_ptr->reserve(pt_num);

    for (int i = 0; i < pt_num; i++){
        LidarPoint point_added;
        point_added.x = - ptr->points[i].y;
        point_added.y = - ptr->points[i].z;
        point_added.z = ptr->points[i].x;
        point_added.normal_x = - ptr->points[i].normal_y;
        point_added.normal_y = - ptr->points[i].normal_z;
        point_added.normal_z = ptr->points[i].normal_x;
        if(std::isnan(point_added.x)||
           std::isnan(point_added.y)||
           std::isnan(point_added.z)||
           std::isnan(point_added.normal_x)||
           std::isnan(point_added.normal_y)||
           std::isnan(point_added.normal_z)){
              continue;
        } 
        new_ptr -> points.push_back(point_added);
    }
    return new_ptr;
}

bool PointCloudProcess::LoadDownsizedMap(double filter_size){
    LidarPointcloudPtr ptr(new LidarPointcloud);

    if (pcl::io::loadPLYFile<LidarPoint>(path_,*ptr) == -1) {
        std::string error = "Couldn't read file " + path_;
        return false;
    }
    std::cout << std::endl;
    std::cout << "Read "
		<< ptr->width * ptr->height
		<< " points from "
        << path_ << std::endl;
    std::cout << std::endl;

    global_pcd_ptr_ = PointCloudDirectionTrans(ptr);
    
    // Point cloud down sample
    pcl::VoxelGrid<LidarPoint> voxel_scan;
    LidarPointcloudPtr new_ptr(new LidarPointcloud());
    voxel_scan.setLeafSize(filter_size,filter_size,filter_size);
    voxel_scan.setInputCloud(global_pcd_ptr_);
    voxel_scan.filter(*new_ptr);
    downsized_map_ptr_ = new_ptr;

    return true;
}

LidarPointcloudPtr PointCloudProcess::GetDownsizedMap(){
    return downsized_map_ptr_;
}

bool PointCloudProcess::SearchNearestNeiborByKdtree(const Eigen::Vector3d& point_3d, Eigen::Vector6d& l_pt){
    LidarPoint feature_point;
    feature_point.getVector3fMap() = point_3d.cast<float>();
    LidarPoint closest_point;
    if (kdtree_ptr_->GetClosestPoint(feature_point,closest_point)){
        l_pt.block(0,0,3,1) = closest_point.getVector3fMap().cast<double>();
        l_pt.block(3,0,3,1) << static_cast<double>(closest_point.normal_x),
                               static_cast<double>(closest_point.normal_y),
                               static_cast<double>(closest_point.normal_z);
        return true;
    } else {
        return false;
    }
}

} //namespace lidar
} //namespace colmap