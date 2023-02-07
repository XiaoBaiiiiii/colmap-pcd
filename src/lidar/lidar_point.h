#ifndef COLMAP_LIDAR_LIDAR_POINT_H
#define COLMAP_LIDAR_LIDAR_POINT_H

#include <Eigen/Core>
#include "util/types.h"
#include <iostream>
namespace colmap{

enum class LidarPointType{Proj, Icp, IcpGround};
class LidarPoint{
  public:
    LidarPoint(LidarPointType type, Eigen::Vector3d& xyz, Eigen::Vector4d& abcd);
    LidarPoint(Eigen::Vector3d& xyz, Eigen::Vector4d& abcd);
    ~LidarPoint(){}

    void SetType(LidarPointType type);
    inline void SetColor(Eigen::Vector3ub& color){color_ = color;}
    inline void SetDist(const double& dist) { dist_ = dist; }
    inline void SetAngle(const double& angle) {angle_ = angle;} 

    inline Eigen::Vector3d LidarXYZ(){
        return xyz_;
    }

    inline const Eigen::Vector3d LidarXYZ() const {
        return xyz_;
    }

    inline Eigen::Vector4d LidarABCD(){
        return abcd_;
    }
    LidarPointType Type();
    inline const double Dist() { return dist_; }
    inline const double Angle() { return angle_; }
    inline const Eigen::Vector3ub& Color() const {return color_;}
    inline Eigen::Vector3ub& Color() {return color_;}

    const double ComputeDist(Eigen::Vector3d& pt);
    const double ComputePointToPointDist(Eigen::Vector3d& pt);
    const double ComputeAngle(Eigen::Vector3d& pt);

    void Normalize();
  private:
    LidarPointType type_;
    Eigen::Vector3d xyz_;
    Eigen::Vector4d abcd_;
    Eigen::Vector3ub color_;// point color
    double dist_; // The residual between the point and point_3d
    double angle_; // The Angle between the normal vector abc and point_3d
};
} // colmap

#endif