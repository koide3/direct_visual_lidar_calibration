#pragma once

#include <camera/generic_camera_base.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/calib/cost_calculator.hpp>

namespace vlcal {

struct NIDCostParams {
  NIDCostParams();
  ~NIDCostParams();

  int bins;         ///< Number of histogram bins for NID computation
};

class CostCalculatorNID : public CostCalculator{
public:
  CostCalculatorNID(const camera::GenericCameraBase::ConstPtr& proj, const VisualLiDARData::ConstPtr& data, const NIDCostParams& params = NIDCostParams());
  virtual ~CostCalculatorNID() override;

  virtual double calculate(const Eigen::Isometry3d& T_camera_lidar) override;

private:
  const NIDCostParams params;
  const camera::GenericCameraBase::ConstPtr proj;
  const VisualLiDARData::ConstPtr data;
  const double max_fov;
};

}  // namespace vlcal
