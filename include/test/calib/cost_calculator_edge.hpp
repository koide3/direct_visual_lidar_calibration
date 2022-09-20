#pragma once

#include <camera/generic_camera_base.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/calib/cost_calculator.hpp>
#include <vlcal/calib/edge_extraction.hpp>
#include <vlcal/calib/nearest_neighbor_search.hpp>

namespace vlcal {

struct EdgeCostParams {
  EdgeCostParams();
  ~EdgeCostParams();

  int num_threads;

  double init_max_correspondence_dist;
  double dec_max_correspondence_dist;
  double min_max_correspondence_dist;

  DepthContinuousEdgeExtractionParams edge_params;
};

class CostCalculatorEdge : public CostCalculator {
public:
  CostCalculatorEdge(const camera::GenericCameraBase::ConstPtr& proj, const VisualLiDARData::ConstPtr& data, const EdgeCostParams& params = EdgeCostParams());
  virtual ~CostCalculatorEdge() override;

  virtual void update_correspondences(const Eigen::Isometry3d& T_camera_lidar) override;
  virtual double calculate(const Eigen::Isometry3d& T_camera_lidar) override;

private:
  const EdgeCostParams params;
  const camera::GenericCameraBase::ConstPtr proj;
  const double max_fov;
  const VisualLiDARData::ConstPtr data;

  double max_correspondence_dist;

  gtsam_ext::FrameCPU::Ptr edges;

  cv::Mat edge_image;
  std::unique_ptr<NearestNeighborSearch> nn_search;

  std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>> correspondences;
};

}  // namespace vlcal
