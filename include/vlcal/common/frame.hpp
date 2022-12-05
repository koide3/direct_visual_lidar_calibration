#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace vlcal {

struct Frame {
public:
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

  Frame()
  : num_points(0),
    times(nullptr),
    points(nullptr),
    normals(nullptr),
    covs(nullptr),
    intensities(nullptr) {}
  virtual ~Frame() {}

  size_t size() const { return num_points; }

  bool has_times() const;        ///< Check if the frame has per-point timestamps
  bool has_points() const;       ///< Check if the frame has points
  bool has_normals() const;      ///< Check if the frame has point normals
  bool has_covs() const;         ///< Check if the frame has point covariances
  bool has_intensities() const;  ///< Check if the frame has point intensities

  bool check_times() const;        ///< Warn if the frame doesn't have times
  bool check_points() const;       ///< Warn if the frame doesn't have points
  bool check_normals() const;      ///< Warn if the frame doesn't have normals
  bool check_covs() const;         ///< Warn if the frame doesn't have covs
  bool check_intensities() const;  ///< Warn if the frame doesn't have intensities

  /**
   * @brief Get the pointer to an aux attribute
   * @param  attrib Attribute name
   * @return If the attribute exists, returns the pointer to it. Otherwise, returns nullptr.
   */
  template <typename T>
  const T* aux_attribute(const std::string& attrib) const {
    const auto found = aux_attributes.find(attrib);
    if (found == aux_attributes.end()) {
      std::cerr << "warning: attribute " << attrib << " not found!!" << std::endl;
      return nullptr;
    }

    if (sizeof(T) != found->second.first) {
      std::cerr << "warning: attribute element size mismatch!! attrib:" << attrib << " size:" << found->second.first << " requested:" << sizeof(T)
                << std::endl;
    }

    return static_cast<const T*>(found->second.second);
  }


public:
  size_t num_points;  ///< Number of points

  double* times;             ///< Per-point timestamp w.r.t. the first point (should be sorted)
  Eigen::Vector4d* points;   ///< Point coordinates (x, y, z, 1)
  Eigen::Vector4d* normals;  ///< Point normals (nx, ny, nz, 0)
  Eigen::Matrix4d* covs;     ///< Point covariances cov(3, 3) = 0
  double* intensities;       ///< Point intensities

  /// Aux attributes <attribute_name, pair<element_size, data_ptr>>
  std::unordered_map<std::string, std::pair<size_t, void*>> aux_attributes;

};
}  // namespace gtsam_ext

#include <vlcal/common/frame_traits.hpp>

namespace vlcal {
namespace frame {

template <>
struct traits<Frame> {
  static int size(const Frame& frame) { return frame.size(); }

  static bool has_times(const Frame& frame) { return frame.has_times(); }
  static bool has_points(const Frame& frame) { return frame.has_points(); }
  static bool has_normals(const Frame& frame) { return frame.has_normals(); }
  static bool has_covs(const Frame& frame) { return frame.has_covs(); }
  static bool has_intensities(const Frame& frame) { return frame.has_intensities(); }

  static double time(const Frame& frame, size_t i) { return frame.times[i]; }
  static const Eigen::Vector4d& point(const Frame& frame, size_t i) { return frame.points[i]; }
  static const Eigen::Vector4d& normal(const Frame& frame, size_t i) { return frame.normals[i]; }
  static const Eigen::Matrix4d& cov(const Frame& frame, size_t i) { return frame.covs[i]; }
  static double intensity(const Frame& frame, size_t i) { return frame.intensities[i]; }

  static const Eigen::Vector4d* points_ptr(const Frame& frame) { return frame.points; }
};

}  // namespace frame
}  // namespace gtsam_ext
