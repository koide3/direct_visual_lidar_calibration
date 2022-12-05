#pragma once

#include <vlcal/common/frame.hpp>
#include <camera/generic_camera_base.hpp>

namespace vlcal {

/**
 * @brief Estimate the bearing vector of a given 2D point
 * @param proj   Camera model
 * @param pt_2d  2D point
 * @return       3D bearing vector
 */
Eigen::Vector3d estimate_direction(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2d& pt_2d);

/**
 * @brief Estimate camera fov
 * @param proj        Camera model
 * @param image_size  Image size
 * @return            FoV [rad]
 */
double estimate_camera_fov(const camera::GenericCameraBase::ConstPtr& proj, const Eigen::Vector2i& image_size);

/**
 * @brief Estimate LiDAR fov
 * @param points  LiDAR points
 * @return        LiDAR FoV [rad]
 */
double estimate_lidar_fov(const Frame::ConstPtr& points);

}  // namespace vlcal
