// Copyright (c) 2023, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#include "controllers/bundle_adjustment.h"

#include <ceres/ceres.h>

#include "optim/bundle_adjustment.h"
#include "util/misc.h"

namespace colmap {
namespace {

// Callback functor called after each bundle adjustment iteration.
class BundleAdjustmentIterationCallback : public ceres::IterationCallback {
 public:
  explicit BundleAdjustmentIterationCallback(Thread* thread)
      : thread_(thread) {}

  virtual ceres::CallbackReturnType operator()(
      const ceres::IterationSummary& summary) {
    CHECK_NOTNULL(thread_);
    thread_->BlockIfPaused();
    if (thread_->IsStopped()) {
      return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    } else {
      return ceres::SOLVER_CONTINUE;
    }
  }

 private:
  Thread* thread_;
};

}  // namespace

BundleAdjustmentController::BundleAdjustmentController(
    const OptionManager& options, Reconstruction* reconstruction)
    : options_(options), reconstruction_(reconstruction) {}
// bundle adjustment批优化
void BundleAdjustmentController::Run() {
  CHECK_NOTNULL(reconstruction_);

  PrintHeading1("Global bundle adjustment");

  const std::vector<image_t>& reg_image_ids = reconstruction_->RegImageIds();

  if (reg_image_ids.size() < 2) {
    std::cout << "ERROR: Need at least two views." << std::endl;
    return;
  }

  // Avoid degeneracies in bundle adjustment.
  reconstruction_->FilterObservationsWithNegativeDepth();

  BundleAdjustmentOptions ba_options = *options_.bundle_adjustment;
  ba_options.solver_options.minimizer_progress_to_stdout = true;
  
  BundleAdjustmentIterationCallback iteration_callback(this);
  ba_options.solver_options.callbacks.push_back(&iteration_callback);

  // Configure bundle adjustment.
  BundleAdjustmentConfig ba_config;
  for (const image_t image_id : reg_image_ids) {
    ba_config.AddImage(image_id);
  }

  if (ba_options.if_add_lidar_constraint){
    ClearLidarPoints();
    IncrementalMapperOptions mapper_options = *options_.mapper;
    std::string path = mapper_options.lidar_pointcloud_path;
    LoadPointcloud(path, mapper_options.PcdProjector());
    std::unordered_set<point3D_t> reg_point3D_ids = reconstruction_->Point3DIds();
    for (point3D_t point3d_id : reg_point3D_ids) {

      ba_config.AddVariablePoint(point3d_id);
      
      Point3D& point3D = reconstruction_->Point3D(point3d_id);
      Eigen::Vector3d pt_xyz = point3D.XYZ();
      Eigen::Vector6d lidar_pt;
      if (lidar_pointcloud_process_->SearchNearestNeiborByKdtree(pt_xyz,lidar_pt)){
        Eigen::Vector3d norm = lidar_pt.block(3,0,3,1);
        Eigen::Vector3d l_pt = lidar_pt.block(0,0,3,1);
        double d = 0 - l_pt.dot(norm);
        Eigen::Vector4d plane;
        plane << norm(0),norm(1),norm(2),d;
        LidarPoint lidar_point(l_pt,plane);
        const double dist2plane = lidar_point.ComputeDist(pt_xyz);
        const double dist2point = lidar_point.ComputePointToPointDist(pt_xyz);
        
        if (dist2plane > 1 || dist2point > 2) continue;
        if (std::abs(norm(1)/norm(0))>10 && std::abs(norm(1)/norm(2))>10) {
          lidar_point.SetType(LidarPointType::IcpGround);
          Eigen::Vector3ub color;
          color << 255,255,0;
          lidar_point.SetColor(color);
        } else {
          lidar_point.SetType(LidarPointType::Icp);
          Eigen::Vector3ub color;
          color << 0,0,255;
          lidar_point.SetColor(color);
        }
        ba_config.AddLidarPoint(point3d_id,lidar_point);
        reconstruction_ -> AddLidarPointInGlobal(point3d_id,lidar_point);

      }
    }
  } else {
    ba_config.SetConstantPose(reg_image_ids[0]);
    ba_config.SetConstantTvec(reg_image_ids[1], {0});
  }

  // Run bundle adjustment.
  BundleAdjuster bundle_adjuster(ba_options, ba_config);
  const BundleAdjuster::OptimazePhrase phrase = BundleAdjuster::OptimazePhrase::WholeMap;
  bundle_adjuster.SetOptimazePhrase(phrase);

  bundle_adjuster.Solve(reconstruction_);

  GetTimer().PrintMinutes();
}

void BundleAdjustmentController::LoadPointcloud(std::string& pointcloud_path, 
                                       const lidar::PcdProjectionOptions& pp_options){
  lidar_pointcloud_process_.reset(new lidar::PointCloudProcess(pointcloud_path));
  if (!lidar_pointcloud_process_->Initialize(pp_options)){
    std::cout<< "Point cloud initialize has error"<<std::endl;
  }
}

void BundleAdjustmentController::ClearLidarPoints(){
  reconstruction_->ClearLidarPoints();
  reconstruction_->ClearLidarPointsInGlobal();
}

}  // namespace colmap
