#include "lidar/lidar_point.h"

namespace colmap
{
LidarPoint::LidarPoint(LidarPointType type, Eigen::Vector3d& xyz, Eigen::Vector4d& abcd){
    type_ = type;
    xyz_ = xyz;
    abcd_ = abcd;
    Normalize();
}

LidarPoint::LidarPoint(Eigen::Vector3d& xyz, Eigen::Vector4d& abcd){
    xyz_ = xyz;
    abcd_ = abcd;
    Normalize();
}

void LidarPoint::SetType(LidarPointType type){
    type_ = type;
}
const double LidarPoint::ComputeDist(Eigen::Vector3d& pt){
    Eigen::Vector3d abc = abcd_.block(0,0,3,1);
    double dist = std::abs(pt.dot(abc) + abcd_(3,0));
    return dist;
}

const double LidarPoint::ComputePointToPointDist(Eigen::Vector3d& pt){
    double dist = (xyz_ - pt).norm();
    return dist;
}

const double LidarPoint::ComputeAngle(Eigen::Vector3d& pt){
    Eigen::Vector3d xyz2pt = pt - xyz_;
    Eigen::Vector3d abc = abcd_.block(0,0,3,1);
    double angle = std::abs(abc.dot(xyz2pt) /xyz2pt.norm());
    return angle;
}

void LidarPoint::Normalize(){
    double a = abcd_(0);
    double b = abcd_(1);
    double c = abcd_(2);
    double norm = sqrt(pow(a,2)+pow(b,2)+pow(c,2));

    a = a/norm;
    b = b/norm;
    c = c/norm;
    double d = 0 - a * xyz_(0) - b * xyz_(1) - c * xyz_(2);
    abcd_ << a, b, c, d;
}

LidarPointType LidarPoint::Type(){
    return type_;
}
} // namespace colmap
