#pragma once

#include <camera/generic_camera.hpp>
#include <vlcal/common/visual_lidar_data.hpp>
#include <vlcal/calib/cost_calculator.hpp>

namespace vlcal {

struct NIDParams {
  NIDParams();
  ~NIDParams();

  int bins;
  int num_threads;
};

class CostCalculatorNID : public CostCalculator{
public:
  CostCalculatorNID(const camera::GenericCameraBase::ConstPtr& proj, const VisualLiDARData::ConstPtr& data, const NIDParams& params = NIDParams());
  virtual ~CostCalculatorNID() override;

  virtual double calculate(const Eigen::Isometry3d& T_camera_lidar) override;

private:
  const NIDParams params;
  const camera::GenericCameraBase::ConstPtr proj;
  const VisualLiDARData::ConstPtr data;
};

}  // namespace vlcal
