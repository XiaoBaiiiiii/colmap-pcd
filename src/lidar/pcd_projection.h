#ifndef COLMAP_LIDAR_PCD_PROJECTION_H
#define COLMAP_LIDAR_PCD_PROJECTION_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
#include <set>
#include <utility>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/pcl_base.h>
#include <pcl/impl/pcl_base.hpp>
#include <mutex>

#include "base/camera.h"
#include "base/database.h"
#include "base/image.h"
#include "base/point2d.h"
#include "base/point3d.h"
#include "base/similarity_transform.h"
#include "base/track.h"

#include "lidar/pt_type.h"

namespace colmap{
namespace lidar{

struct PcdProjectionOptions {
  double depth_image_scale = 0.2;
  bool if_save_depth_image = false;
  std::string depth_image_folder = "/Users/baixuxing/code/colmap/25-photos/test";
  std::string original_image_folder;
  bool if_save_lidar_frame = false;
  std::string lidar_frame_folder = "/Users/baixuxing/code/colmap/25-photos/test_nodes_points";
  int max_proj_scale = 10;
  int min_proj_scale = 2;
  double min_proj_dist = 2;
  float submap_length = 1.0;
  float submap_width = 1.0;
  float submap_height = 1.0;
  float choose_meter = 40.0;// The z axis length of the pyramid
  double min_lidar_proj_dist;

};
class PcdProj{
  public:
    using PointType = LidarPoint;
    using KeyType = Eigen::Matrix<int,3,1>;
    using NodeType = std::vector<PointType,Eigen::aligned_allocator<PointType>>;
    using MapType = pcl::PointCloud<PointType>::Ptr;
    using PlaneType = Eigen::Vector4f;
    using ImageMapType = std::vector<NodeType*>;

    explicit PcdProj(PcdProjectionOptions options) : options_(options){}
    ~PcdProj(){}
    void BuildSubMap(const MapType& ptr);
    void SetNewImage(const Image& image, 
                     const Camera& camera, 
                     std::map<point3D_t,Eigen::Matrix<double,6,1>>& map);
    void SetNewImage(const Image& image, 
            const Camera& camera, 
            std::vector<std::pair<Eigen::Vector2d, bool>,Eigen::aligned_allocator<std::pair<Eigen::Vector2d, bool>>>& pt_xys, 
            std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>>& pt_xyzs);
    inline KeyType GetKeyType(const PointType& pt){
      Eigen::Vector3f pt_cor = pt.getVector3fMap();
      KeyType key;
      key << round(pt_cor(0)/options_.submap_length),
          round(pt_cor(1)/options_.submap_height),round(pt_cor(2)/options_.submap_width);
      return key;
    }

    struct compare{
      bool operator()(KeyType const& a, KeyType const& b) const {
        if (a(0)!=b(0)){
          return a(0) < b(0);
        } else if (a(1)!=b(1)){
          return a(1) < b(1);
        } else {
          return a(2) < b(2);
        }
      }
    };

    struct fea_compare{
      bool operator()(Eigen::Matrix<int,2,1> const& a, Eigen::Matrix<int,2,1> const& b) const {
        if (a(0)!=b(0)){
          return a(0) < b(0);
        } else {
          return a(1) < b(1);
        }
      }
    };

    struct LImage {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        LImage(std::set<Eigen::Matrix<int,2,1>,fea_compare> feat,Eigen::Matrix3d& r,Eigen::Vector3d& t){
          feature_points = feat;
          rot_cw = r; 
          t_cw = t;
        }
        std::string img_name; 
        int img_height; 
        int img_width; 
        Eigen::Matrix3d rot_cw;
        Eigen::Vector3d t_cw;
        double cx; 
        double cy;
        double fx;
        double fy;
        int succeed_match = 0;
        // Pixel coordinates of feature points
        std::set<Eigen::Matrix<int,2,1>,fea_compare> feature_points;
        // Distance between image pixel point and corresponding lidar point
        std::map<Eigen::Matrix<int,2,1>,float,fea_compare> dist_map;
        // Distance between image feature points and corresponding lidar points
        std::map<Eigen::Matrix<int,2,1>, std::pair<PointType, float>, fea_compare> feature_pts_map;
    };
    
  private:
    // Pyramid struct
    struct QuadPyramid{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      QuadPyramid(Eigen::Vector3f vertex, Eigen::Vector3f a, Eigen::Vector3f b, 
          Eigen::Vector3f c, Eigen::Vector3f d):
          vertex(vertex),corner_1(a),corner_2(b),corner_3(c),corner_4(d){

          plane_0 << GetPlane(corner_1, corner_4, corner_3);
          plane_1 << GetPlane(vertex, corner_1, corner_2);
          plane_2 << GetPlane(vertex, corner_2, corner_3);
          plane_3 << GetPlane(vertex, corner_3, corner_4);
          plane_4 << GetPlane(vertex, corner_4, corner_1);
      }

      PlaneType GetPlane(Eigen::Vector3f& a, Eigen::Vector3f& b, Eigen::Vector3f& c){
          Eigen::Vector3f ab = a - b;
          Eigen::Vector3f ac = a - c;
          Eigen::Vector3f normal = ab.cross(ac);
          float d = -(normal(0)*a(0) + normal(1)*a(1) + normal(2)*a(2));
          PlaneType result(normal(0), normal(1), normal(2), d);
          return result;
      }

      Eigen::Vector3f vertex;
      Eigen::Vector3f corner_1;
      Eigen::Vector3f corner_2;
      Eigen::Vector3f corner_3;
      Eigen::Vector3f corner_4;

      PlaneType plane_0;
      PlaneType plane_1;
      PlaneType plane_2;
      PlaneType plane_3;
      PlaneType plane_4;

    };  

    void SearchSubMap(const LImage& img, ImageMapType& image_map);
    // void SearchSubMap(const TestImage& img, ImageMapType& image_map);
    void ImageMapProj(LImage& img, ImageMapType& image_map, const Camera& camera);
    // void ImageMapProj(TestImage& img, ImageMapType& image_map);
    // void SetTestNewImage(const Image& image, const Camera& camera);
    void SaveDepthImage(const LImage& img);
    // Search the nodes in the pyramid
    void SearchImageMap(QuadPyramid& quad, ImageMapType& image_map);
    // Distort model of the opencv model
    Eigen::Vector2d DistortOpenCV(Eigen::Vector2d& ori_uv, const Camera& camera);

    MapType global_map_ptr_; // ptr to the whole point cloud map
    PcdProjectionOptions options_;
    float global_map_min_x_ = std::numeric_limits<float>::max();
    float global_map_max_x_ = std::numeric_limits<float>::min();
    float global_map_min_y_ = std::numeric_limits<float>::max();
    float global_map_max_y_ = std::numeric_limits<float>::min();
    float global_map_min_z_ = std::numeric_limits<float>::max();
    float global_map_max_z_ = std::numeric_limits<float>::min();
    int submap_num_ = 0;

    std::map<KeyType,NodeType,compare> submap_;

    std::mutex proj_mutex_;

};
} //namespace lidar
} //namespace colmap
#endif
