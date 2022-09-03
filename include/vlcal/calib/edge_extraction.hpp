#pragma once

#include <gtsam_ext/types/frame_cpu.hpp>

namespace vlcal {

struct DepthContinuousEdgeExtractionParams {
  DepthContinuousEdgeExtractionParams();
  ~DepthContinuousEdgeExtractionParams();

  double voxel_resolution;
  int min_voxel_points;
  int min_plane_points;

  int ransac_max_iterations;
  double ransac_dist_thresh;

  double plane_angle_min;
  double plane_angle_max;

  double edge_sampling_step;
};

class DepthContinuousEdgeExtraction {
public:
  DepthContinuousEdgeExtraction(const DepthContinuousEdgeExtractionParams& params);
  ~DepthContinuousEdgeExtraction();

  gtsam_ext::FrameCPU::Ptr extract(const gtsam_ext::Frame::ConstPtr& points, std::vector<gtsam_ext::FrameCPU::Ptr>* plane_points = nullptr);

private:
  const DepthContinuousEdgeExtractionParams params;
};

}  // namespace vlcal
