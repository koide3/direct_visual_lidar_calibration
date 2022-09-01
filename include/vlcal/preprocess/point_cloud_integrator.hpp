#pragma once

#include <gtsam_ext/types/frame.hpp>

namespace vlcal {

class PointCloudIntegrator {
public:
  virtual ~PointCloudIntegrator() {}

  virtual void insert_points(const gtsam_ext::Frame::ConstPtr& raw_points) = 0;
  virtual gtsam_ext::Frame::ConstPtr get_points() = 0;
};

}