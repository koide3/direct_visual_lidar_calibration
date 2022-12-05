#include <vlcal/common/frame.hpp>

#include <fstream>
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/iterator/counting_iterator.hpp>

namespace vlcal {

bool Frame::has_times() const {
  return times;
}

bool Frame::has_points() const {
  return points;
}

bool Frame::has_normals() const {
  return normals;
}

bool Frame::has_covs() const {
  return covs;
}

bool Frame::has_intensities() const {
  return intensities;
}

bool Frame::check_times() const {
  if (!times) {
    std::cerr << "warning: frame doesn't have times" << std::endl;
  }
  return times;
}

bool Frame::check_points() const {
  if (!points) {
    std::cerr << "warning: frame doesn't have points" << std::endl;
  }
  return points;
}

bool Frame::check_normals() const {
  if (!normals) {
    std::cerr << "warning: frame doesn't have normals" << std::endl;
  }
  return normals;
}

bool Frame::check_covs() const {
  if (!covs) {
    std::cerr << "warning: frame doesn't have covs" << std::endl;
  }
  return covs;
}

bool Frame::check_intensities() const {
  if (!intensities) {
    std::cerr << "warning: frame doesn't have intensities" << std::endl;
  }
  return intensities;
}

}  // namespace gtsam_ext
